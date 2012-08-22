/* * IAP - Interactive Perception System - Segment visually observable environment
 * into rigid bodies and estimate type and properties of joints between them by
 * means of interaction.
 * Copyright (C) 2012 Technische Universitaet Berlin - RBO
 * <robotics@robotics.tu-berlin.de>
 * 
 * This file is part of IAP.
 * 
 * IAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * IAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with IAP.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * BundlerInitializer.cpp
 *
 *  Created on: 06.08.2010
 *      Author: dermax
 */
#include "BundlerInitializer.h"
#include "../libs/imagelib/util.h"
#include "../libs/matrix/matrix.h"
#include "../libs/imagelib/triangulate.h"

#include "ros/ros.h"

using namespace SFM;

BundlerInitializer::BundlerInitializer(vision::CameraModel &camera, double min_motion_pix) :
  Initializer(camera, min_motion_pix), BundlerApp()
{
  use_last_frame = false;
  num_settings = 9;
}

bool BundlerInitializer::OnInit()
{
  return true;
}

void BundlerInitializer::ProcessOptions(int argc, char **argv)
{
}

std::vector<vision::FeaturePtr> BundlerInitializer::run(std::vector<std::vector<vision::FeaturePtr> > &features)
{
  std::vector<vision::FeaturePtr> result;
  if (features[0].size() < 20)
  {
    for (std::vector<vision::FeaturePtr>::iterator it = features[0].begin(); it != features[0].end(); it++)
    {
      // The Estimator needs the 3D coordinates in this format:
      // Origin = Bottom-left corner of the image
      // Origin' = Center of the image
      // Direction +x = Direction -u
      // Direction +y = Direction +v
      // Direction +z = Behind the image
      vision::FeaturePtr f = (*it)->cloneAndUpdate(
                                                   -(float)(((*it)->getX() - cam.getXresolution() / 2.0) * default_depth
                                                       / cam.getFocalLength()),
                                                   (float)(((*it)->getY() - cam.getYresolution() / 2.0) * default_depth
                                                       / cam.getFocalLength()), this->default_depth);
      result.push_back(f);
    }
    ROS_WARN("[BundlerInitializer::run] Not enough features for bundler, using default depth");
    return result;
  }

  std::vector<std::vector<vision::FeaturePtr> > bundler_frames = selectBundlerFrames(features);

  int num_bundler_frames = bundler_frames.size();
  ROS_INFO( "[BundlerInitializer::run] Using %d frames for Bundler", num_bundler_frames);
  m_fixed_focal_length = true;
  m_run_bundle = true;
  m_init_focal_length = cam.getFocalLength();
  m_matches = MatchTable(num_bundler_frames);
  RemoveAllMatches();
  m_image_data.clear();

  int frame_num = 0;
  for (std::vector<std::vector<vision::FeaturePtr> >::iterator frame = bundler_frames.begin(); frame
      != bundler_frames.end(); frame++)
  {
    if (frame_num > 0)
    {
      SetMatch(frame_num - 1, frame_num);
      int nMatches = frame->size();
      /* Read the matches */
      std::vector<KeypointMatch> matches;
      for (int i = 0; i < nMatches; i++)
      {
        int k1, k2;
        k1 = i;
        k2 = i;
        KeypointMatch m;

        m.m_idx1 = k1;
        m.m_idx2 = k2;

        matches.push_back(m);
      }

      ::MatchIndex idx = GetMatchIndex(frame_num - 1, frame_num);
      // m_match_lists[idx] = matches;
      m_matches.GetMatchList(idx) = matches;
    }
    ::ImageData data;
    for (std::vector<vision::FeaturePtr>::iterator feat = frame->begin(); feat != frame->end(); feat++)
    {
      ::Keypoint key;
      key.m_x = (float)((*feat)->getX() - cam.getXresolution() / 2.0);
      key.m_y = (float)((*feat)->getY() - cam.getYresolution() / 2.0);
      data.m_keys.push_back(key);
    }
    data.m_width = cam.getXresolution();
    data.m_height = cam.getYresolution();
    data.m_cached_dimensions = true;
    data.m_keys_loaded = true;
    // focal length stuff
    data.m_has_init_focal = true;
    data.m_init_focal = cam.getFocalLength();
    data.m_image_loaded = true;

    m_image_data.push_back(data);
    frame_num++;
  }
  m_matches_computed = true;
  m_num_original_images = num_bundler_frames;

  //		BundleAdjust();
  //############################################
  int num_images = GetNumImages();
  int new_image_start = 0;

  ComputeEpipolarGeometry(true, new_image_start);
  ComputeTransforms(false, new_image_start);

  MakeMatchListsSymmetric();

  ComputeTracks(new_image_start);

  RemoveAllMatches();

  /* Set match flags */
  int num_tracks = (int)m_track_data.size();
  for (int i = 0; i < num_tracks; i++)
  {
    TrackData &track = m_track_data[i];
    int num_views = (int)track.m_views.size();

    for (int j = 0; j < num_views; j++)
    {
      int img1 = track.m_views[j].first;

      assert(img1 >= 0 && img1 < num_images);

      for (int k = j + 1; k < num_views; k++)
      {
        int img2 = track.m_views[k].first;

        assert(img2 >= 0 && img2 < num_images);

        SetMatch(img1, img2);
        SetMatch(img2, img1);
      }
    }
  }

  // ######## from BundlerApp::BundleAdjustFast()
  /* Compute initial image information */
  //ComputeGeometricConstraints();

  /* Set track pointers to -1 */
  for (int i = 0; i < (int)m_track_data.size(); i++)
  {
    m_track_data[i].m_extra = -1;
  }

  /* For now, assume all images form one connected component */
  //int num_images = GetNumImages();
  int *added_order = new int[num_images];
  int *added_order_inv = new int[num_images];

  /* **** Run bundle adjustment! **** */

  camera_params_t *cameras = new camera_params_t[num_images];
  int max_pts = (int)m_track_data.size(); // 1243742; /* HACK! */
  v3_t *points = new v3_t[max_pts];
  v3_t *colors = new v3_t[max_pts];
  std::vector<ImageKeyVector> pt_views;

  /* Initialize the bundle adjustment */
  int num_init_cams = 0;
  InitializeBundleAdjust(num_init_cams, added_order, added_order_inv, cameras, points, colors, pt_views,
                         m_use_constraints);

  int i_best = -1, j_best = -1, max_matches = 0;
  double max_score = 0.0;
  int curr_num_cameras, curr_num_pts;
  int pt_count;

  if (num_init_cams == 0)
  {
    BundlePickInitialPair(i_best, j_best, true);

    added_order[0] = i_best;
    added_order[1] = j_best;

    /* **** Set up the initial cameras **** */
    double init_focal_length_0 = 0.0, init_focal_length_1 = 0.0;
    pt_count = curr_num_pts = SetupInitialCameraPair(i_best, j_best, init_focal_length_0, init_focal_length_1, cameras,
                                                     points, colors, pt_views);

    /* Run sfm for the first time */
    double error0;
    error0 = RunSFM(curr_num_pts, 2, 0, false, cameras, points, added_order, colors, pt_views);

    if (m_fix_necker)
    {
      /* Swap the cameras and flip the depths to deal with Necker
       * reversal */

      camera_params_t cameras_old[2];
      v3_t *points_old;

      points_old = new v3_t[curr_num_pts];

      memcpy(points_old, points, sizeof(v3_t) * curr_num_pts);
      memcpy(cameras_old, cameras, sizeof(camera_params_t) * 2);

      camera_params_t tmp = cameras[0];
      memcpy(cameras[0].R, cameras[1].R, sizeof(double) * 9);
      memcpy(cameras[0].t, cameras[1].t, sizeof(double) * 3);
      cameras[0].f = init_focal_length_0;
      cameras[0].k[0] = cameras[0].k[1] = 0.0;

      memcpy(cameras[1].R, tmp.R, sizeof(double) * 9);
      memcpy(cameras[1].t, tmp.t, sizeof(double) * 3);
      cameras[1].f = init_focal_length_1;
      cameras[1].k[0] = cameras[1].k[1] = 0.0;

      double K1inv[9] = {1.0 / cameras[0].f, 0.0, 0.0, 0.0, 1.0 / cameras[0].f, 0.0, 0.0, 0.0, 1.0};

      double K2inv[9] = {1.0 / cameras[1].f, 0.0, 0.0, 0.0, 1.0 / cameras[1].f, 0.0, 0.0, 0.0, 1.0};

      for (int i = 0; i < curr_num_pts; i++)
      {
        int k1 = pt_views[i][0].second;
        int k2 = pt_views[i][1].second;

        double proj1[3] = {GetKey(added_order[0], k1).m_x, GetKey(added_order[0], k1).m_y, -1.0};

        double proj2[3] = {GetKey(added_order[1], k2).m_x, GetKey(added_order[1], k2).m_y, -1.0};

        double proj1_norm[3], proj2_norm[3];

        matrix_product(3, 3, 3, 1, K1inv, proj1, proj1_norm);
        matrix_product(3, 3, 3, 1, K2inv, proj2, proj2_norm);

        v2_t p = v2_new(proj1_norm[0] / proj1_norm[2], proj1_norm[1] / proj1_norm[2]);

        v2_t q = v2_new(proj2_norm[0] / proj2_norm[2], proj2_norm[1] / proj2_norm[2]);

        double proj_error;

        double t1[3];
        double t2[3];

        /* Put the translation in standard form */
        matrix_product(3, 3, 3, 1, cameras[0].R, cameras[0].t, t1);
        matrix_scale(3, 1, t1, -1.0, t1);
        matrix_product(3, 3, 3, 1, cameras[1].R, cameras[1].t, t2);
        matrix_scale(3, 1, t2, -1.0, t2);

        points[i] = triangulate(p, q, cameras[0].R, t1, cameras[1].R, t2, &proj_error);
      }

      double error1;
      error1 = RunSFM(curr_num_pts, 2, 0, false, cameras, points, added_order, colors, pt_views);

    }

    curr_num_cameras = 2;
  }
  else
  {

    curr_num_cameras = num_init_cams;
    pt_count = curr_num_pts = (int)m_point_data.size();
  }

  int round = 0;
  while (curr_num_cameras < num_images)
  {
    int parent_idx;
    int max_cam = FindCameraWithMostMatches(curr_num_cameras, curr_num_pts, added_order, parent_idx, max_matches,
                                            pt_views);

    if (max_matches < m_min_max_matches)
      break; /* No more connections */

    /* Find all images with 90% of the matches of the maximum */
    std::vector<ImagePair> image_set;

    if (false && max_matches < 48)
    {
      image_set.push_back(ImagePair(max_cam, parent_idx));
    }
    else
    {
      // int nMatches = MIN(100, iround(0.75 /* 0.9 */ * max_matches));
      int nMatches = iround(0.75 /* 0.9 */* max_matches);
      image_set = FindCamerasWithNMatches(nMatches, curr_num_cameras, curr_num_pts, added_order, pt_views);
    }

    int num_added_images = (int)image_set.size();

    /* Now, throw the new cameras into the mix */
    int image_count = 0;
    for (int i = 0; i < num_added_images; i++)
    {
      int next_idx = image_set[i].first;
      int parent_idx = image_set[i].second;

      added_order[curr_num_cameras + image_count] = next_idx;

      /* **** Set up the new camera **** */
      bool success = false;
      camera_params_t camera_new = BundleInitializeImage(m_image_data[next_idx], next_idx,
                                                         curr_num_cameras + image_count, curr_num_cameras,
                                                         curr_num_pts, added_order, points,
                                                         NULL /*cameras + parent_idx*/, cameras, pt_views, &success);

      if (success)
      {
        cameras[curr_num_cameras + image_count] = camera_new;
        image_count++;
      }
      else
      {
        ROS_WARN("[BundlerInitializer::run] Couldn't initialize image %d", next_idx);
        m_image_data[next_idx].m_ignore_in_bundle = true;
      }
    }

    /* Compute the distance between the first pair of cameras */
    double dist0 = 0.0;

    pt_count = curr_num_pts;

    curr_num_cameras += image_count;

    if (!m_skip_add_points)
    {
      pt_count = BundleAdjustAddAllNewPoints(pt_count, curr_num_cameras, added_order, cameras, points, colors, dist0,
                                             pt_views);
    }

    curr_num_pts = pt_count;

    if (!m_skip_full_bundle)
    {
      /* Run sfm again to update parameters */
      RunSFM(curr_num_pts, curr_num_cameras, 0, false, cameras, points, added_order, colors, pt_views);

      /* Remove bad points and cameras */
      RemoveBadPointsAndCameras(curr_num_pts, curr_num_cameras + 1, added_order, cameras, points, colors, pt_views);
    }

    round++;
  }

  if (m_estimate_ignored)
  {
    EstimateIgnoredCameras(curr_num_cameras, cameras, added_order, curr_num_pts, points, colors, pt_views);
  }

  // end of BundlerApp::BundleAdjustFast()

  int idx = -1;
  for (int j = 0; j < curr_num_cameras; j++)
  {
    if (added_order[j] == 0)
    {
      idx = j;
      break;
    }
  }

  double t[3];
  if (idx == -1)
  {
    // no result
    return result;
  }
  else
  {
    // calculate translation vector
    matrix_product(3, 3, 3, 1, cameras[idx].R, cameras[idx].t, t);
    matrix_scale(3, 1, t, -1.0, t);
  }
  // generate result
  double point[3], cur_point[3];
  double ba_z_sum = 0;
  for (int i = 0; i < curr_num_pts; i++)
  {
    // Check if the point is visible in any view
    if ((int)pt_views[i].size() != 0)
    {
      cur_point[0] = Vx(points[i]);
      cur_point[1] = Vy(points[i]);
      cur_point[2] = Vz(points[i]);
      matrix_product(3, 3, 3, 1, cameras[idx].R, cur_point, point);
      vision::FeaturePtr f(
                           new vision::Feature((float)(point[0] + t[0]), (float)(point[1] + t[1]),
                                               (float)(point[2] + t[2])));
      ba_z_sum += (float)(point[2] + t[2]);
      f->setId((*(features.at(0).begin() + i))->getId());
      result.push_back(f);
    }
  }

  // Normalize to have a median depth z=1 and rescale all points accordingly
  double ba_scale = ((double)result.size()) / ba_z_sum;
  std::vector<vision::FeaturePtr>::iterator res_it = result.begin();
  std::vector<vision::FeaturePtr>::iterator res_it_end = result.end();
  // The Estimator needs the 3D coordinates in this format:
  // Direction +x' = Direction +x
  // Direction +y' = Direction -y
  // Direction +z' = Direction +z
  for(; res_it!=res_it_end; res_it++)
  {
    (*res_it)->setPos((*res_it)->getX() * ba_scale ,  -(*res_it)->getY() * ba_scale, (*res_it)->getZ() * ba_scale);
  }
  return result;
}

void BundlerInitializer::setBundlerFrames(std::vector<int>& frames)
{
  bundler_frame_index = frames;
}

std::vector<std::vector<vision::FeaturePtr> > BundlerInitializer::selectBundlerFrames(
                                                                                      std::vector<std::vector<
                                                                                          vision::FeaturePtr> > features)
{
  std::vector<std::vector<vision::FeaturePtr> > bundler_frames;
  // check if bundler frames where set externaly
  if (bundler_frame_index.size() >= 2)
  {
    // use frames from "bundler_frame_index" set by setBundlerFrames()
    std::vector<int>::iterator it = bundler_frame_index.begin();
    std::vector<int>::iterator it_end = bundler_frame_index.end();
    for (; it != it_end; it++)
    {
      assert(*it >= 0);
      assert(*it < features.size());
      bundler_frames.push_back(features[*it]);
    }
    return bundler_frames;
  }

  // calculate the frames based on the motion threshold
  // save last used frame (always use first frame)
  std::vector<vision::FeaturePtr> last_used_frame = features[0];
  bundler_frames.push_back(last_used_frame);
  int f = 0;
  for (std::vector<std::vector<vision::FeaturePtr> >::iterator it = features.begin(); it != features.end(); it++)
  {
    double dif = absDif(*it, last_used_frame);
    // dif is calculated in pixels, motion_threshold is in 1/xresolution
    if (dif > motion_threshold * cam.getXresolution())
    {
      last_used_frame = *it;
      // save frame features
      bundler_frames.push_back(last_used_frame);
      ROS_INFO( "[BundlerInitializer::selectBundlerFrames] using frame: %d", f);
    }
    f++;
  }
  if (use_last_frame)
  {
    // use last frame
    bundler_frames.push_back(*(features.end() - 1));
  }
  return bundler_frames;
}

/**
 * Try different settings for the motion threshold
 * We could not find one setting that worked for all experiments
 * these settings worked for some experiments
 */
void BundlerInitializer::setSetting(int num)
{
  switch (num)
  {
    case 1:
      motion_threshold = 30.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
    case 2:
      motion_threshold = 35.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
    case 3:
      motion_threshold = 50.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
    case 4:
      motion_threshold = 60.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
    case 5:
      motion_threshold = 75.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
    case 6:
      motion_threshold = 80.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
    case 7:
      motion_threshold = 90.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
    case 8:
      motion_threshold = 100.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
    default:
    case 0:
      motion_threshold = 30.0 / (double)cam.getXresolution();
      use_last_frame = true;
      break;
  }
}
