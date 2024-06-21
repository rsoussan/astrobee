/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <interest_point/BAD.h>
#include <interest_point/brisk.h>
#include <interest_point/HashSIFT.h>
#include <interest_point/matching.h>
#include <opencv2/xfeatures2d.hpp>
#include <localization_common/timer.h>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <vector>
// Note: if any of these values are manually set by the user in
// build_map, the localize script must be invoked with precisely the
// same settings!
// TODO(oalexan1): Ideally the settings used here must be saved in the
// map file, for the localize executable to read them from there.
DEFINE_int32(hamming_distance, 100,
             "A smaller value keeps fewer but more reliable binary descriptor matches.");
DEFINE_double(goodness_ratio, 0.8,
              "A smaller value keeps fewer but more reliable float descriptor matches.");
DEFINE_int32(orgbrisk_octaves, 4,
             "Number of octaves, or scale spaces, that BRISK will evaluate.");
DEFINE_double(orgbrisk_pattern_scale, 1.0,
             "The pattern scale to use for BRISK.");

// Customize the feature detectors.
DEFINE_int32(detection_retries, 5,
             "Number of attempts to acquire the desired number of features with the detector.");
// SURF detector
DEFINE_int32(min_surf_features, 200,
             "Minimum number of features to be computed using SURF.");
DEFINE_int32(max_surf_features, 800,
             "Maximum number of features to be computed using SURF.");
DEFINE_double(min_surf_threshold, 5,
              "Minimum threshold for feature detection using SURF.");
DEFINE_double(default_surf_threshold, 10000,
              "Default threshold for feature detection using SURF.");
DEFINE_double(max_surf_threshold, 1000000,
              "Maximum threshold for feature detection using SURF.");
// ORGBRISK detector
DEFINE_int32(min_brisk_features, 400,
             "Minimum number of features to be computed using ORGBRISK.");
DEFINE_int32(max_brisk_features, 1000000,
             "Maximum number of features to be computed using ORGBRISK.");
DEFINE_double(min_brisk_threshold, 1,
              "Minimum threshold for feature detection using ORGBRISK.");
DEFINE_double(default_brisk_threshold, 30,
              "Default threshold for feature detection using ORGBRISK.");
DEFINE_double(max_brisk_threshold, 110,
              "Maximum threshold for feature detection using ORGBRISK.");

double hamming_, ratio_;

namespace interest_point {

  DynamicDetector::DynamicDetector(int min_features, int max_features, int max_retries,
                                   double min_thresh, double default_thresh, double max_thresh):
    min_features_(min_features), max_features_(max_features), max_retries_(max_retries),
    min_thresh_(min_thresh), default_thresh_(default_thresh), max_thresh_(max_thresh),
    dynamic_thresh_(default_thresh) {}

  void DynamicDetector::GetDetectorParams(int & min_features, int & max_features, int & max_retries,
                                          double & min_thresh, double & default_thresh,
                                          double & max_thresh) {
    min_features = min_features_; max_features = max_features_; max_retries = max_retries_;
    min_thresh = min_thresh_; default_thresh = default_thresh_; max_thresh = max_thresh_;
  }

  void DynamicDetector::Detect(const cv::Mat& image,
                               std::vector<cv::KeyPoint>* keypoints,
                               cv::Mat* keypoints_description) {
    // Sometimes we want a placeholder detector for initialization. Yet
    // that one cannot be used until it is configured.
    if (default_thresh_ <= 0)
      LOG(FATAL) << "The detector parameters have not been set.";

    for (unsigned int i = 0; i < max_retries_; i++) {
      keypoints->clear();
      DetectImpl(image, keypoints);
      if (keypoints->size() < min_features_)
        TooFew();
      else if (keypoints->size() > max_features_)
        TooMany();
      else
        break;
    }

      static localization_common::Timer mt("detect timer");
    mt.Start();
    ComputeImpl(image, keypoints, keypoints_description);
    mt.StopAndLog();
  }

  class BriskDynamicDetector : public DynamicDetector {
   public:
    BriskDynamicDetector(int min_features, int max_features, int max_retries,
                         double min_thresh, double default_thresh, double max_thresh)
      : DynamicDetector(min_features, max_features, max_retries,
                        min_thresh, default_thresh, max_thresh) {
      std::cout << "set brisk with default thresh: " << default_thresh << std::endl;
      Reset();
    }

    void Reset(void) {
      brisk_ = interest_point::BRISK::create(dynamic_thresh_, FLAGS_orgbrisk_octaves,
                                 FLAGS_orgbrisk_pattern_scale);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      brisk_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      brisk_->compute(image, *keypoints, *keypoints_description);
    }
    virtual void TooMany(void) {
      dynamic_thresh_ *= 1.25;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= 0.8;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ < min_thresh_)
        dynamic_thresh_ = min_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }

   private:
    cv::Ptr<interest_point::BRISK> brisk_;
  };

  class SurfDynamicDetector : public DynamicDetector {
   public:
    SurfDynamicDetector(int min_features, int max_features, int max_retries,
                        double min_thresh, double default_thresh, double max_thresh)
      : DynamicDetector(min_features, max_features, max_retries,
                        min_thresh, default_thresh, max_thresh) {
      std::cout << "setting surf dyn thresho: " << dynamic_thresh_ << std::endl;
      surf_ = cv::xfeatures2d::SURF::create(dynamic_thresh_);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      surf_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      static localization_common::Timer mt("surf detect timer");
      mt.Start();
      surf_->compute(image, *keypoints, *keypoints_description);
      mt.StopAndLog();
    }
    virtual void TooMany(void) {
      dynamic_thresh_ *= 1.5;
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      std::cout << "too many!: " << dynamic_thresh_ << std::endl;
      surf_->setHessianThreshold(static_cast<float>(dynamic_thresh_));
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= 0.9;
      if (dynamic_thresh_ < min_thresh_)
        dynamic_thresh_ = min_thresh_;
      std::cout << "too few!: " << dynamic_thresh_ << std::endl;
      surf_->setHessianThreshold(static_cast<float>(dynamic_thresh_));
    }

   private:
    cv::Ptr<cv::xfeatures2d::SURF> surf_;
  };
  /*class SurfDynamicDetector : public DynamicDetector {
   public:
    SurfDynamicDetector(int min_features, int max_features, int max_retries,
                        double min_thresh, double default_thresh, double max_thresh)
      : DynamicDetector(min_features, max_features, max_retries,
                        min_thresh, default_thresh, max_thresh) {
      Reset();
    }

    void Reset(void) {
      surf_ = cv::xfeatures2d::SURF::create(dynamic_thresh_);
      brisk_ = interest_point::BRISK::create(dynamic_thresh_, FLAGS_orgbrisk_octaves,
                                 FLAGS_orgbrisk_pattern_scale);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      brisk_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      surf_->compute(image, *keypoints, *keypoints_description);
    }
    virtual void TooMany(void) {
      dynamic_thresh_ *= 1.25;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= 0.8;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ < min_thresh_)
        dynamic_thresh_ = min_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }

   private:
    cv::Ptr<cv::xfeatures2d::SURF> surf_;
    cv::Ptr<interest_point::BRISK> brisk_;
  };*/



  class HashSIFTDynamicDetector : public DynamicDetector {
   public:
    HashSIFTDynamicDetector(int min_features, int max_features, int max_retries,
                        double min_thresh, double default_thresh, double max_thresh)
      : DynamicDetector(min_features, max_features, max_retries,
                        min_thresh, default_thresh, max_thresh) {
      Reset();
    }

    void Reset(void) {
      hash_sift_ = upm::HashSIFT::create(5.0, upm::HashSIFT::SIZE_256_BITS);
      brisk_ = interest_point::BRISK::create(dynamic_thresh_, FLAGS_orgbrisk_octaves,
                                 FLAGS_orgbrisk_pattern_scale);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      brisk_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      hash_sift_->compute(image, *keypoints, *keypoints_description);
    }
    virtual void TooMany(void) {
      dynamic_thresh_ *= 1.25;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ > max_thresh_) {
      std::cout << "too many!: " << dynamic_thresh_ << std::endl;
        dynamic_thresh_ = max_thresh_;
      }
      brisk_->setThreshold(dynamic_thresh_);
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= 0.8;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ < min_thresh_) {
      std::cout << "too few!: " << dynamic_thresh_ << std::endl;
        dynamic_thresh_ = min_thresh_;
      }
      brisk_->setThreshold(dynamic_thresh_);
    }

   private:
    cv::Ptr<cv::Feature2D> hash_sift_;
    cv::Ptr<interest_point::BRISK> brisk_;
  };

  class BadDynamicDetector : public DynamicDetector {
   public:
    BadDynamicDetector(int min_features, int max_features, int max_retries,
                        double min_thresh, double default_thresh, double max_thresh)
      : DynamicDetector(min_features, max_features, max_retries,
                        min_thresh, default_thresh, max_thresh) {
      Reset();
    }

    void Reset(void) {
      bad_ = upm::BAD::create(5.0, upm::BAD::SIZE_256_BITS);
      brisk_ = interest_point::BRISK::create(dynamic_thresh_, FLAGS_orgbrisk_octaves,
                                 FLAGS_orgbrisk_pattern_scale);
    }

    virtual void DetectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints) {
      brisk_->detect(image, *keypoints);
    }
    virtual void ComputeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>* keypoints,
                             cv::Mat* keypoints_description) {
      bad_->compute(image, *keypoints, *keypoints_description);
    }
    virtual void TooMany(void) {
      dynamic_thresh_ *= 1.25;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ > max_thresh_)
        dynamic_thresh_ = max_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }
    virtual void TooFew(void) {
      dynamic_thresh_ *= 0.8;
      dynamic_thresh_ = static_cast<int>(dynamic_thresh_);  // for backwards compatibility
      if (dynamic_thresh_ < min_thresh_)
        dynamic_thresh_ = min_thresh_;
      brisk_->setThreshold(dynamic_thresh_);
    }

   private:
    cv::Ptr<cv::Feature2D> bad_;
    cv::Ptr<interest_point::BRISK> brisk_;
  };



  FeatureDetector::FeatureDetector(std::string const& detector_name,
                                   int min_features, int max_features, int retries,
                                   double min_thresh, double default_thresh, double max_thresh) {
    detector_ = NULL;
    Reset(detector_name, min_features, max_features, retries,
          min_thresh, default_thresh, max_thresh);
  }

  void FeatureDetector::GetDetectorParams(int & min_features, int & max_features, int & max_retries,
                                          double & min_thresh, double & default_thresh,
                                          double & max_thresh) {
    if (detector_ == NULL)
      LOG(FATAL) << "The detector was not set.";
    detector_->GetDetectorParams(min_features, max_features, max_retries,
                                 min_thresh, default_thresh, max_thresh);
  }

  FeatureDetector::~FeatureDetector(void) {
    if (detector_ != NULL) {
      delete detector_;
      detector_ = NULL;
    }
  }

  void FeatureDetector::Reset(std::string const& detector_name, int min_features, int max_features, int retries,
                              double min_thresh, double default_thresh, double max_thresh, double hamming,
                              double ratio) {
    hamming_ = hamming;
    ratio_ = ratio;
    detector_name_ = detector_name;

    if (detector_ != NULL) {
      delete detector_;
      detector_ = NULL;
    }

    // Populate the defaults
    if (max_features <= 0) {
      // SURF and HASHSIFT both use SURF to detect features
      if (detector_name == "notSURF") {
        min_features   = FLAGS_min_surf_features;
        max_features   = FLAGS_max_surf_features;
        retries        = FLAGS_detection_retries;
        min_thresh     = FLAGS_min_surf_threshold;
        default_thresh = FLAGS_default_surf_threshold;
        max_thresh     = FLAGS_max_surf_threshold;
      } else if (detector_name == "ORGBRISK" || detector_name == "SURF") {
        min_features   = FLAGS_min_brisk_features;
        max_features   = FLAGS_max_brisk_features;
        retries        = FLAGS_detection_retries;
        min_thresh     = FLAGS_min_brisk_threshold;
        default_thresh = FLAGS_default_brisk_threshold;
        max_thresh     = FLAGS_max_brisk_threshold;
      } else {
        LOG(FATAL) << "Unimplemented feature detector: " << detector_name;
      }
    }

    // Loading the detector
    if (detector_name == "ORGBRISK")
      detector_ = new BriskDynamicDetector(min_features, max_features, retries,
                                           min_thresh, default_thresh, max_thresh);
    else if (detector_name == "SURF")
      detector_ = new SurfDynamicDetector(min_features, max_features, retries,
                                          min_thresh, default_thresh, max_thresh);
    else if (detector_name == "HASHSIFT")
      detector_ = new HashSIFTDynamicDetector(min_features, max_features, retries,
                                          min_thresh, default_thresh, max_thresh);
    else if (detector_name == "BAD")
      detector_ = new BadDynamicDetector(min_features, max_features, retries,
                                          min_thresh, default_thresh, max_thresh);
    else
      LOG(FATAL) << "Unimplemented feature detector: " << detector_name;

    LOG(INFO) << "Using descriptor: " << detector_name;
  }

  void FeatureDetector::Detect(const cv::Mat& image,
                               std::vector<cv::KeyPoint>* keypoints,
                               cv::Mat* keypoints_description) {
    if (detector_ == NULL)
      LOG(FATAL) << "The detector was not initialized.";

    detector_->Detect(image, keypoints, keypoints_description);

    // Normalize the image points relative to the center of the image
    for (cv::KeyPoint& key : *keypoints) {
    //  key.pt.x -= image.cols/2.0;
     // key.pt.y -= image.rows/2.0;
    }
  }

  void FindMatches(const cv::Mat & img1_descriptor_map,
                   const cv::Mat & img2_descriptor_map, std::vector<cv::DMatch> * matches) {
    std::cout << "d1 depth: " << img1_descriptor_map.depth() << std::endl;
    std::cout << "d2 depth: " << img2_descriptor_map.depth() << std::endl;
    CHECK(img1_descriptor_map.depth() ==
          img2_descriptor_map.depth())
      << "Mixed descriptor types. Did you mash BRISK with SIFT/SURF?";

    // Check for early exit conditions
    matches->clear();
    if (img1_descriptor_map.rows == 0 ||
        img2_descriptor_map.rows == 0)
      return;

    if (false) {  // img1_descriptor_map.depth() == CV_8U) {
      static localization_common::Timer mt("matching timer");
      mt.Start();
      // Binary descriptor

      cv::BFMatcher matcher(cv::NORM_HAMMING, true  /* Forward & Backward matching */);
      // cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::LshIndexParams>(3, 18, 2));
      matcher.match(img1_descriptor_map, img2_descriptor_map, *matches);

      // Select only inlier matches that meet a BRISK threshold of
      // of FLAGS_hamming_distance.
      // TODO(oalexan1) This needs further study.
      std::vector<cv::DMatch> inlier_matches;
      inlier_matches.reserve(matches->size());  // This saves time in allocation
      for (cv::DMatch const& dmatch : *matches) {
        // std::cout << "matching binary! hamming: " << hamming_ << ", distance: " << dmatch.distance << std::endl;
        if (dmatch.distance < hamming_) {
          inlier_matches.push_back(dmatch);
        }
      }
      matches->swap(inlier_matches);  // Doesn't invoke a copy of all elements.
      mt.StopAndLog();
    } else {
      static localization_common::Timer mt("matching timer");
      mt.Start();
      // Traditional floating point descriptor
      std::cout << "matching surf! hamming: " << hamming_ << ", ratio: " << ratio_ << std::endl;
      // cv::FlannBasedMatcher matcher;
      cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::LshIndexParams>(3, 18, 2));
      std::vector<std::vector<cv::DMatch> > possible_matches;
      matcher.knnMatch(img1_descriptor_map, img2_descriptor_map, possible_matches, 2);
      matches->clear();
      matches->reserve(possible_matches.size());
      for (std::vector<cv::DMatch> const& best_pair : possible_matches) {
        if (best_pair.size() == 0) continue;
        // std::cout << "surf match distance: " << best_pair.at(0).distance << std::endl;
        // std::cout << "hamming: " << hamming_ << std::endl;
        if (best_pair.at(0).distance > hamming_) continue;
        if (best_pair.size() == 1) {
          // This was the only best match, push it.
          matches->push_back(best_pair.at(0));
        } else {
          // std::cout << "ratio: " << best_pair.at(0).distance/static_cast<double>(best_pair.at(1).distance) <<
          // std::endl;
          // Push back a match only if it is 25% better than the next best.
          if (best_pair.at(0).distance < ratio_ * best_pair.at(1).distance) {
            matches->push_back(best_pair[0]);
          }
        }
      }
      mt.StopAndLog();
    }
  }
}  // namespace interest_point
