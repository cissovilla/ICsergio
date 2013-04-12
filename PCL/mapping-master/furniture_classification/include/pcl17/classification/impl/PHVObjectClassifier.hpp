/*
 * PHVObjectClassifier.hpp
 *
 *  Created on: Jun 9, 2012
 *      Author: vsu
 */

#ifndef PHVOBJECTCLASSIFIER_HPP_
#define PHVOBJECTCLASSIFIER_HPP_

#include <pcl17/classification/PHVObjectClassifier.h>
#include <opencv2/core/core.hpp>
#include <pcl17/features/vfh.h>

template<class FeatureT>
cv::Mat transform_to_mat(const std::vector<FeatureT> & features) {
	int featureLength = sizeof(features[0].histogram) / sizeof(float);

	cv::Mat res(features.size(), featureLength, CV_32F);
	for (size_t i = 0; i < features.size(); i++) {
		for (int j = 0; j < featureLength; j++) {
			res.at<float>(i, j) = features[i].histogram[j];
		}
	}

	return res;
}

template<class FeatureT>
void transform_to_features(const cv::Mat & mat,
		std::vector<FeatureT> & features) {
	features.clear();
	for (int i = 0; i < mat.rows; i++) {
		FeatureT f;
		for (int j = 0; j < mat.cols; j++) {
			f.histogram[j] = mat.at<float>(i, j);
		}
		features.push_back(f);
	};
}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::clusterFeatures(
		vector<FeatureT> & cluster_centers, vector<int> & cluster_labels) {
	int featureLength = sizeof(features_[0].histogram) / sizeof(float);

	cv::Mat feature_vectors = transform_to_mat(features_);
	cv::Mat centers(num_clusters_, featureLength, feature_vectors.type()),
			labels;

	cv::kmeans(feature_vectors, num_clusters_, labels,
			cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 4,
			cv::KMEANS_RANDOM_CENTERS, centers);

	transform_to_features(centers, cluster_centers);
	cluster_labels = labels;
}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::saveToFile() {

	// Delete old and create new directory sturcture for output
	boost::filesystem::path output_path(database_dir_);
	boost::filesystem::path output_models_path(database_dir_ + "models/");
	if (boost::filesystem::exists(output_path)) {
		boost::filesystem::remove_all(output_path);
	}

	boost::filesystem::create_directories(output_path);
	boost::filesystem::create_directories(output_models_path);

	YAML::Emitter out;

	out << *this;

	std::ofstream f;
	f.open((database_dir_ + "database.yaml").c_str());
	f << out.c_str();
	f.close();

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::loadFromFile() {

	std::ifstream fin((database_dir_ + "database.yaml").c_str());
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	doc >> *this;

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::computeClassifier() {

	BOOST_FOREACH(ModelMapValueType &v, class_name_to_partial_views_map_) {
		int view_num_counter = 0;

		for (size_t i = 0; i < v.second.size(); i++) {
			appendFeaturesFromCloud(v.second[i], v.first, view_num_counter);
			view_num_counter++;

		}

	}

	// Transform to model centroinds in local coordinate frame of the segment
	centroids_.getMatrixXfMap() *= -1;

	normalizeFeatures(features_);

	if (debug_) {

		for (int i = 0; i < num_clusters_; i++) {
			std::stringstream ss;
			ss << debug_folder_ << "Cluster" << i << "/";

			boost::filesystem::path output_path(ss.str());
			if (boost::filesystem::exists(output_path)) {
				boost::filesystem::remove_all(output_path);
			}

			boost::filesystem::create_directories(output_path);
		}

		std::ofstream f((debug_folder_ + "features.txt").c_str());

		size_t N = sizeof(features_[0].histogram) / sizeof(float);

		for (size_t i = 0; i < features_.size(); i++) {
			for (size_t j = 0; j < N; j++) {
				f << features_[i].histogram[j] << " ";
			}

			f << "\n";

		}

		f.close();

		std::ofstream f2((debug_folder_ + "classnames.txt").c_str());

		for (size_t i = 0; i < classes_.size(); i++) {

			f2 << classes_[i] << "\n";

		}

		f2.close();

		std::ofstream f3((debug_folder_ + "centroids.txt").c_str());

		for (size_t i = 0; i < centroids_.size(); i++) {

			f3 << centroids_.points[i].x << " " << centroids_.points[i].y << " "
					<< centroids_.points[i].z << "\n";

		}

		f3.close();

	}

	vector<FeatureT> cluster_centers;
	vector<int> cluster_labels;

	clusterFeatures(cluster_centers, cluster_labels);

	database_.clear();

	for (size_t i = 0; i < cluster_labels.size(); i++) {

		FeatureT cluster_center = cluster_centers[cluster_labels[i]];
		std::string classname = classes_[i];

		database_[cluster_center][classname].points.push_back(centroids_[i]);
		database_[cluster_center][classname].width =
				database_[cluster_center][classname].points.size();
		database_[cluster_center][classname].height = 1;
		database_[cluster_center][classname].is_dense = true;

		//ransac_result_threshold_[classname] = 0.01;

		if (debug_) {
			std::stringstream ss;
			ss << debug_folder_ << "Cluster" << cluster_labels[i] << "/Segment"
					<< i << ".pcd";
			pcl17::io::savePCDFileASCII(ss.str(), *segment_pointclouds_[i]);

		}

	}

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::computeExternalClassifier(
		const std::string & labels) {

	BOOST_FOREACH(ModelMapValueType &v, class_name_to_partial_views_map_) {
		int view_num_counter = 0;

		for (size_t i = 0; i < v.second.size(); i++) {
			appendFeaturesFromCloud(v.second[i], v.first, view_num_counter);
			view_num_counter++;

		}

	}

	// Transform to model centroinds in local coordinate frame of the segment
	centroids_.getMatrixXfMap() *= -1;

	normalizeFeatures(features_);

	for (int i = 0; i < num_clusters_; i++) {
		std::stringstream ss;
		ss << debug_folder_ << "Cluster" << i << "/";

		boost::filesystem::path output_path(ss.str());
		if (boost::filesystem::exists(output_path)) {
			boost::filesystem::remove_all(output_path);
		}

		boost::filesystem::create_directories(output_path);
	}

	std::ofstream f((debug_folder_ + "features.txt").c_str());

	size_t N = sizeof(features_[0].histogram) / sizeof(float);

	for (size_t i = 0; i < features_.size(); i++) {
		for (size_t j = 0; j < N; j++) {
			f << features_[i].histogram[j] << " ";
		}

		f << "\n";

	}

	f.close();

	std::ofstream f2((debug_folder_ + "classnames.txt").c_str());

	for (size_t i = 0; i < classes_.size(); i++) {

		f2 << classes_[i] << "\n";

	}

	f2.close();

	std::ofstream f3((debug_folder_ + "centroids.txt").c_str());

	for (size_t i = 0; i < centroids_.size(); i++) {

		f3 << centroids_.points[i].x << " " << centroids_.points[i].y << " "
				<< centroids_.points[i].z << "\n";

	}

	f3.close();

	vector<int> cluster_labels;
	set<int> unique_cluster_labels;
	vector<FeatureT> cluster_centers;

	std::ifstream f4(labels.c_str());

	pcl17::PointCloud<FeatureT> mat;
	vector<float> bias;

	int feature_size = sizeof(features_[0].histogram) / sizeof(float);

	std::string line;
	while (std::getline(f4, line))
	//while(f4)

	{

		// now we'll use a stringstream to separate the fields out of the line
		//std::stringstream ss( line );
		//std::string field;
		FeatureT ff;
		//int i=0;

		std::vector<std::string> st;
		boost::split(st, line, boost::is_any_of(","), boost::token_compress_on);

		//std::cerr << "Feature size " << feature_size << " Vector size " << st.size() <<  std::endl;

		bias.push_back(atof(st[0].c_str()));

		//while (std::getline( ss, field, ',' ))
		for (size_t i = 0; i < feature_size; i++) {
			// for each field we wish to convert it to a double
			// (since we require that the CSV contains nothing but floating-point values)

			//std::stringstream fs( field );
			//float v = 0;
			//fs >> v;
			//float v;
			//char tmp;
			//f4 >> v;
			//f4 >> tmp;
			//std::cerr << i << " " << st[i] << std::endl;
			ff.histogram[i] = atof(st[i + 1].c_str());
			//i++;
		}
		mat.points.push_back(ff);
		mat.width = mat.points.size();
		mat.height = 1;
	}

	//std::cerr << "Matrix:\n" << mat << std::endl;

	//  while(f4)
	//  {
	//    FeatureT ff;
	//    for(int i=0; i<feature_size; i++)
	//    {
	//      if (f4.peek() == ',')
	//      {
	//        f4.ignore();
	//      }
	//      f4 >> ff.histogram[i];
	//    }
	//    mat.push_back(ff);
	//  }
	//
	//  std::ofstream f5("mat.txt");
	//  for(int i=0; i<mat.points.size(); i++)
	//  {
	//    for(int j=0; j<feature_size; i++)
	//    {
	//      f5 << mat.points[i].histogram[j] << ",";
	//    }
	//    f5 << "\n";
	//  }
	//  f5.close();

	//TODO check code

	int cluster_size = mat.size();
	num_clusters_ = cluster_size;

	for (size_t i = 0; i < features_.size(); i++) {
		Eigen::ArrayXf res = Eigen::ArrayXf::Zero(cluster_size);

		for (size_t j = 0; j < cluster_size; j++) {
			res[j] += bias[j];
			for (size_t k = 0; k < feature_size; k++) {
				res[j] += mat[j].histogram[k] * features_[i].histogram[k];
			}
		}

		int cluster_idx;
		res.maxCoeff(&cluster_idx);
		//std::cerr << "Cluster number " << cluster_idx << std::endl;
		FeatureT ff;

		for (size_t k = 0; k < feature_size; k++) {
			ff.histogram[k] = 0;
		}

		ff.histogram[0] = cluster_idx;

		database_[ff][classes_[i]].push_back(centroids_.points[i]);
		ransac_result_threshold_[classes_[i]] = 0.5;

		if (debug_) {
			std::stringstream ss;
			ss << debug_folder_ << "Cluster" << cluster_idx << "/Segment" << i
					<< ".pcd";
			pcl17::io::savePCDFileASCII(ss.str(), *segment_pointclouds_[i]);

		}

	}

	external_classifier_ = labels;

}

template<class PointT, class PointNormalT, class FeatureT>
furniture_classification::Hypothesis::Ptr pcl17::PHVObjectClassifier<PointT,
		PointNormalT, FeatureT>::generate_hypothesis(
		std::map<std::string, pcl17::PointCloud<pcl17::PointXYZ>::Ptr> & votes_map) {

	features_.clear();
	centroids_.clear();
	classes_.clear();
	votes_map.clear();

	votes_.clear();
	voted_segment_idx_.clear();

	appendFeaturesFromCloud(scene_, "Scene", 0);
	normalizeFeaturesWithCurrentMinMax(features_);
	vote();

	furniture_classification::Hypothesis::Ptr hp(
			new furniture_classification::Hypothesis);

	for (std::map<std::string, pcl17::PointCloud<pcl17::PointXYZI> >::const_iterator it =
			votes_.begin(); it != votes_.end(); it++) {

		int grid_center_x, grid_center_y;

		//std::cerr << it->first << " " << it->second.size() << " votes" << std::endl;

		Eigen::MatrixXf grid = projectVotesToGrid(it->second, grid_center_x,
				grid_center_y);

		PointCloudPtr local_maxima_ = findLocalMaximaInGrid(grid, window_size_);

		//std::cerr << it->first << " local max " << local_maxima_->size() << std::endl;

		local_maxima_->header.frame_id = "/base_link";
		votes_map[it->first] = local_maxima_;

		for (int i = 0; i < local_maxima_->points.size(); i++) {
			geometry_msgs::Pose2D p;
			p.x = local_maxima_->points[i].x;
			p.y = local_maxima_->points[i].y;
			hp->poses.push_back(p);
			hp->classes.push_back(it->first);
		}

	}

	return hp;
}

template<class PointT, class PointNormalT, class FeatureT> furniture_classification::FittedModelsPtr pcl17::PHVObjectClassifier<
		PointT, PointNormalT, FeatureT>::fit_objects(
		furniture_classification::Hypothesis::ConstPtr hp) {

	pcl17::IterativeClosestPoint<PointNormalT, PointNormalT> icp;
	//create point to plane transformationEstimation object
	boost::shared_ptr<
			pcl17::registration::TransformationEstimationLM<PointNormalT,
					PointNormalT> > transformEstPointToPlane(
			new pcl17::registration::TransformationEstimationLM<PointNormalT,
					PointNormalT>());
	//icp.setTransformationEstimation(transformEstPointToPlane);
	icp.setMaximumIterations(icp_max_iterations_);
	icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);

	icp.setEuclideanFitnessEpsilon(0);

	//pcl17::IterativeClosestPoint<PointNormalT, PointNormalT> icp;
	icp.setInputTarget(scene_);

	furniture_classification::FittedModelsPtr res(new furniture_classification::FittedModels);
	//PointNormalCloudPtr res(new PointNormalCloud);
	//res->header.frame_id = "/base_link";

	std::map<std::string, vector<PointNormalCloudPtr> > result_vector_map;
	std::map<std::string, vector<float> > scores_map;

	for (int i = 0; i < hp->poses.size(); i++) {

		PointNormalCloudPtr best_fit(new PointNormalCloud);
		double min_fittness = std::numeric_limits<double>::max();

		for (int j = 0;
				j < class_name_to_full_models_map_[hp->classes[i]].size();
				j++) {

			for (int angle_idx = 0; angle_idx < num_angles_; angle_idx++) {

				double angle = (angle_idx * 2 * M_PI) / num_angles_;

				PointNormalCloudPtr cloud_transformed(new PointNormalCloud);

				Eigen::Affine3f transform;
				transform.setIdentity();
				Eigen::Translation<float, 3> translation(hp->poses[i].x,
						hp->poses[i].y, 0);
				Eigen::AngleAxis<float> rotation(angle,
						Eigen::Vector3f(0, 0, 1));
				transform *= rotation;
				transform *= translation;

				pcl17::transformPointCloud(
						*class_name_to_full_models_map_[hp->classes[i]][j],
						*cloud_transformed, transform);

				icp.setInputSource(cloud_transformed);
				PointNormalCloudPtr Final(new PointNormalCloud);
				icp.align(*Final);

				double score = icp.getFitnessScore();
				//std::cerr << i << " " << j << " " << score << std::endl;
				if (score < min_fittness) {
					best_fit = Final;
					min_fittness = score;
				}

			}
		}

		if (min_fittness < ransac_result_threshold_[hp->classes[i]]) {
			result_vector_map[hp->classes[i]].push_back(best_fit);
			scores_map[hp->classes[i]].push_back(min_fittness);
		}

	}

	typename std::map<std::string, vector<PointNormalCloudPtr> >::iterator it;

	for (it = result_vector_map.begin(); it != result_vector_map.end(); it++) {

		vector<float> scores;
		vector<PointNormalCloudPtr> result_vector = removeIntersecting(
				it->second, scores_map[it->first], &scores);

		for (int i = 0; i < result_vector.size(); i++) {
			sensor_msgs::PointCloud2 r;
			pcl17::toROSMsg(*result_vector[i], r);
			res->models.push_back(r);
			res->scores.push_back(scores[i]);
			res->classes.push_back(it->first);
			//*res += *result_vector[i];
		}

	}

	return res;

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::classify() {
	appendFeaturesFromCloud(scene_, "Scene", 0);
	normalizeFeaturesWithCurrentMinMax(features_);
	vote();

	RandomSampleConsensusSimple<PointNormalT> ransac(
			subsampling_resolution_ * 2);

	ransac.setScene(scene_);
	ransac.setMaxIterations(ransac_num_iter_);
	ransac.setWeight(ransac_vis_score_weight_);

	for (std::map<std::string, pcl17::PointCloud<pcl17::PointXYZI> >::const_iterator it =
			votes_.begin(); it != votes_.end(); it++) {

		std::cerr << "Checking for " << it->first << std::endl;
		pcl17::io::savePCDFileASCII(debug_folder_ + it->first + "_votes.pcd",
				it->second);

		int grid_center_x, grid_center_y;
		Eigen::MatrixXf grid = projectVotesToGrid(it->second, grid_center_x,
				grid_center_y);

		if (debug_) {
			Eigen::MatrixXi img = (grid * 255.0 / grid.maxCoeff()).cast<int>();

			std::stringstream filename;
			filename << debug_folder_ << it->first << "_center_"
					<< grid_center_x << "_" << grid_center_y << ".pgm";
			std::ofstream f(filename.str().c_str());
			f << "P2\n" << grid.cols() << " " << grid.rows() << "\n255\n";
			f << img;
		}

		vector<PointNormalCloudPtr> result;
		vector<float> scores;

		BOOST_FOREACH(PointNormalCloudPtr & full_model, class_name_to_full_models_map_[it->first]) {
			PointNormalT minp, maxp;
			pcl17::getMinMax3D<PointNormalT>(*full_model, minp, maxp);
			float window_size = std::max((maxp.x - minp.x), (maxp.y - minp.y))
					/ 2;

			std::cerr << "Window size " << window_size << std::endl;

			ransac.setModel(full_model);

			PointCloudPtr local_maxima_ = findLocalMaximaInGrid(grid,
					window_size);

			if (debug_ && !local_maxima_->empty())
				pcl17::io::savePCDFileASCII(
						debug_folder_ + it->first + "_local_maxima.pcd",
						*local_maxima_);

			vector<boost::shared_ptr<std::vector<int> > > voted_segments;
			voted_segments = findVotedSegments(local_maxima_, it->first,
					window_size);

			fitModelsWithRansac(voted_segments, it->first, ransac, result,
					scores);

		}

		//generateVisibilityScore(result, scores);
		result = removeIntersecting(result, scores);

		found_objects_[it->first] = result;

	}

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::eval_clustering(
		const std::string & classname, const float search_radius, double &tp,
		double &fn, double &fp) {
	appendFeaturesFromCloud(scene_, "Scene", 0);
	normalizeFeaturesWithCurrentMinMax(features_);
	vote();

	for (std::map<std::string, pcl17::PointCloud<pcl17::PointXYZI> >::const_iterator it =
			votes_.begin(); it != votes_.end(); it++) {

		//std::cerr << "Checking for " << it->first << std::endl;
		//pcl17::io::savePCDFileASCII(debug_folder_ + it->first + "_votes.pcd", it->second);

		int grid_center_x, grid_center_y;
		Eigen::MatrixXf grid = projectVotesToGrid(it->second, grid_center_x,
				grid_center_y);

		BOOST_FOREACH (PointNormalCloudPtr & full_model, class_name_to_full_models_map_[it->first]) {
			PointNormalT minp, maxp;
			pcl17::getMinMax3D<PointNormalT>(*full_model, minp, maxp);
			float window_size = std::max((maxp.x - minp.x), (maxp.y - minp.y))
					/ 2;

			Eigen::ArrayXXi local_max = getLocalMaximaGrid(grid, window_size);

			int ctp, cfp, cfn;
			int search_radius_pixels = search_radius / cell_size_;

			if (it->first == classname) {

				Eigen::ArrayXXi true_region = local_max.block(
						grid_center_x - search_radius_pixels,
						grid_center_y - search_radius_pixels,
						2 * search_radius_pixels + 1,
						2 * search_radius_pixels + 1);

				ctp = (true_region == 1).count();
				cfn = (ctp == 0);
				cfp = (local_max == 1).count() - ctp;

			} else {
				cfp = (local_max == 1).count();
				ctp = 0;
				cfn = 0;
			}

			tp += ctp;
			fp += cfp;
			fn += cfn;

		}

	}

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::eval_clustering_external(
		const std::string & classname, const float search_radius, double &tp,
		double &fn, double &fp, const std::string & matrix) {

	features_.clear();
	classes_.clear();
	centroids_.clear();

	appendFeaturesFromCloud(scene_, "Scene", 0);
	normalizeFeaturesWithCurrentMinMax(features_);
	vote_external(matrix);

	for (std::map<std::string, pcl17::PointCloud<pcl17::PointXYZI> >::const_iterator it =
			votes_.begin(); it != votes_.end(); it++) {

		//std::cerr << "Checking for " << it->first << std::endl;
		//pcl17::io::savePCDFileASCII(debug_folder_ + it->first + "_votes.pcd", it->second);

		int grid_center_x, grid_center_y;
		Eigen::MatrixXf grid = projectVotesToGrid(it->second, grid_center_x,
				grid_center_y);

		BOOST_FOREACH (PointNormalCloudPtr & full_model, class_name_to_full_models_map_[it->first]) {
			PointNormalT minp, maxp;
			pcl17::getMinMax3D<PointNormalT>(*full_model, minp, maxp);
			float window_size = std::max((maxp.x - minp.x), (maxp.y - minp.y))
					/ 2;

			Eigen::ArrayXXi local_max = getLocalMaximaGrid(grid, window_size);

			int ctp, cfp, cfn;
			int search_radius_pixels = search_radius / cell_size_;

			if (it->first == classname) {

				Eigen::ArrayXXi true_region = local_max.block(
						grid_center_x - search_radius_pixels,
						grid_center_y - search_radius_pixels,
						2 * search_radius_pixels + 1,
						2 * search_radius_pixels + 1);

				ctp = (true_region == 1).count();
				cfn = (ctp == 0);
				cfp = (local_max == 1).count() - ctp;

			} else {
				cfp = (local_max == 1).count();
				ctp = 0;
				cfn = 0;
			}

			tp += ctp;
			fp += cfp;
			fn += cfn;

		}

	}

}

template<class PointT, class PointNormalT, class FeatureT>
typename pcl17::PointCloud<PointNormalT>::Ptr pcl17::PHVObjectClassifier<PointT,
		PointNormalT, FeatureT>::estimateNormalsAndSubsample(
		typename pcl17::PointCloud<PointT>::ConstPtr cloud_orig) {

	//std::vector<int> idx;
	//PointCloudPtr cloud(new PointCloud);
	//pcl17::removeNaNFromPointCloud(*cloud_orig, *cloud, idx);

	PointNormalCloudPtr cloud_with_normals(new PointNormalCloud);
	PointNormalCloudPtr cloud_downsampled(new PointNormalCloud);

	//    pcl17::VoxelGrid<PointT> grid;
	//    grid.setInputCloud(cloud_orig);
	//    grid.setLeafSize(subsampling_resolution_, subsampling_resolution_, subsampling_resolution_);
	//    grid.filter(*cloud_downsampled);

	PointTreePtr tree(new PointTree);

	mls_->setComputeNormals(true);

	mls_->setInputCloud(cloud_orig);
	mls_->setPolynomialFit(mls_polynomial_fit_);
	mls_->setPolynomialOrder(mls_polynomial_order_);
	mls_->setSearchMethod(tree);
	mls_->setSearchRadius(mls_search_radius_);

	this->mls_->process(*cloud_with_normals);

	pcl17::VoxelGrid<PointNormalT> grid;
	grid.setInputCloud(cloud_with_normals);
	grid.setLeafSize(subsampling_resolution_, subsampling_resolution_,
			subsampling_resolution_);
	grid.filter(*cloud_downsampled);

	Eigen::Vector4f so = cloud_orig->sensor_origin_;
	//std::cerr << "Model viewpoint\n" << so << std::endl;
	for (size_t i = 0; i < cloud_downsampled->points.size(); i++) {

		pcl17::flipNormalTowardsViewpoint(cloud_downsampled->points[i], so[0],
				so[1], so[2], cloud_downsampled->points[i].normal_x,
				cloud_downsampled->points[i].normal_y,
				cloud_downsampled->points[i].normal_z);
	}

	//cloud_downsampled->is_dense = false;
	//pcl17::removeNaNFromPointCloud(*cloud_downsampled, *cloud_downsampled, idx);

	cloud_downsampled->sensor_origin_ = cloud_orig->sensor_origin_;
	cloud_downsampled->sensor_orientation_ = cloud_orig->sensor_orientation_;

	return cloud_downsampled;

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::getSegmentsFromCloud(
		PointNormalCloudPtr cloud_with_normals,
		vector<boost::shared_ptr<vector<int> > > & segment_indices,
		pcl17::PointCloud<pcl17::PointXYZRGBNormal>::Ptr & colored_segments) {

	segment_indices.clear();

	PointCloudPtr cloud(new PointCloud);
	pcl17::PointCloud<pcl17::Normal>::Ptr normals(
			new pcl17::PointCloud<pcl17::Normal>);
	pcl17::copyPointCloud(*cloud_with_normals, *cloud);
	pcl17::copyPointCloud(*cloud_with_normals, *normals);

	PointTreePtr tree(new PointTree);

	vector<pcl17::PointIndices> segment_indices_all;

	pcl17::RegionGrowing<PointT, pcl17::Normal> region_growing;
	region_growing.setMinClusterSize(min_points_in_segment_);
	//region_growing.setMaxClusterSize (10000);
	region_growing.setSearchMethod(tree);
	region_growing.setNumberOfNeighbours(30);
	region_growing.setInputCloud(cloud);
	//region_growing.setIndices (indices);
	region_growing.setInputNormals(normals);
	region_growing.setSmoothnessThreshold(rg_smoothness_threshold_);
	region_growing.setResidualThreshold(rg_residual_threshold_);
	region_growing.setCurvatureThreshold(1.0);
	region_growing.setCurvatureTestFlag(false);
	region_growing.setSmoothModeFlag(false);
	region_growing.setResidualTestFlag(true);

	//segment_indices_all = region_growing.getSegments();
	region_growing.extract(segment_indices_all);

	BOOST_FOREACH(pcl17::PointIndices & i, segment_indices_all) {
		if ((int) i.indices.size() > min_points_in_segment_) {
			boost::shared_ptr<vector<int> > indices(new vector<int>);
			*indices = i.indices;
			segment_indices.push_back(indices);
		}
	}

	if (debug_) {
		pcl17::PointCloud<pcl17::PointXYZRGB>::Ptr colored_segments_all;
		colored_segments_all = region_growing.getColoredCloud();

		vector<int> valid_segment_indices;

		BOOST_FOREACH(boost::shared_ptr<vector<int> > & i, segment_indices) {
			valid_segment_indices.insert(valid_segment_indices.begin(),
					i->begin(), i->end());
		}

		colored_segments.reset(new pcl17::PointCloud<pcl17::PointXYZRGBNormal>);

		for (size_t j = 0; j < valid_segment_indices.size(); j++) {
			pcl17::PointXYZRGBNormal p;
			int i = valid_segment_indices[j];
			p.x = colored_segments_all->points[i].x;
			p.y = colored_segments_all->points[i].y;
			p.z = colored_segments_all->points[i].z;
			p.normal_x = normals->points[i].normal_x;
			p.normal_y = normals->points[i].normal_y;
			p.normal_z = normals->points[i].normal_z;
			p.rgb = colored_segments_all->points[i].rgb;
			colored_segments->push_back(p);
		}

		colored_segments->sensor_origin_ = cloud_with_normals->sensor_origin_;
		colored_segments->sensor_orientation_ =
				cloud_with_normals->sensor_orientation_;

	}

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::appendFeaturesFromCloud(
		PointNormalCloudPtr & cloud, const string & class_name, const int i) {
	if (debug_) {
		std::stringstream ss;
		ss << debug_folder_ << class_name << i << ".pcd";
		std::cerr << "Writing to file " << ss.str() << std::endl;
		pcl17::io::savePCDFileASCII(ss.str(), *cloud);

	}

	pcl17::PointCloud<pcl17::PointXYZRGBNormal>::Ptr colored_segments;
	vector<boost::shared_ptr<vector<int> > > segment_indices;
	getSegmentsFromCloud(cloud, segment_indices, colored_segments);

	segment_indices_ = segment_indices;

	PointNormalTreePtr tree(new PointNormalTree);

	feature_estimator_->setSearchMethod(tree);
	feature_estimator_->setKSearch(fe_k_neighbours_);
	feature_estimator_->setInputCloud(cloud);

	typename pcl17::VFHEstimation<PointNormalT, PointNormalT, FeatureT>::Ptr f =
			boost::dynamic_pointer_cast<
					pcl17::VFHEstimation<PointNormalT, PointNormalT, FeatureT> >(
					feature_estimator_);

	if (f) {
		f->setInputNormals(cloud);
		f->setKSearch(20);
		f->setViewPoint(cloud->sensor_origin_[0], cloud->sensor_origin_[1],
				cloud->sensor_origin_[2]);
	}

	BOOST_FOREACH(const boost::shared_ptr<vector<int> > & idx, segment_indices) { // compute deature for segment
		pcl17::PointCloud<FeatureT> feature;
		//PointNormalCloudPtr p(new PointNormalCloud(*cloud, *idx));
		//feature_estimator_->setInputCloud(p);
		feature_estimator_->setIndices(idx);
		feature_estimator_->compute(feature);

		// Compute centroid of segment
		Eigen::Vector4f centroid;
		pcl17::compute3DCentroid(*cloud, *idx, centroid);
		PointT centroid_point;
		centroid_point.x = centroid[0];
		centroid_point.y = centroid[1];
		centroid_point.z = centroid[2];

		features_.push_back(feature.points[0]);
		centroids_.points.push_back(centroid_point);
		classes_.push_back(class_name);

		if (debug_) {
			PointNormalCloudPtr segm(new PointNormalCloud(*cloud, *idx));
			segment_pointclouds_.push_back(segm);
		}

	}

	centroids_.width = centroids_.points.size();
	centroids_.height = 1;
	centroids_.is_dense = true;

	// if debug dump partial views and segmentations
	if (debug_) {
		std::stringstream ss;
		ss << debug_folder_ << class_name << i << "_segmentation.pcd";
		std::cerr << "Writing to file " << ss.str() << std::endl;
		pcl17::io::savePCDFileASCII(ss.str(), *colored_segments);
	}
}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::normalizeFeatures(
		std::vector<FeatureT> & features) {

	int N = sizeof(min_.histogram) / sizeof(float);

	// Init max and min vales with first feature
	for (int j = 0; j < N; j++) {
		min_.histogram[j] = features[0].histogram[j];
		max_.histogram[j] = features[0].histogram[j];
	}

	// Find max and min values
	for (size_t i = 0; i < features.size(); i++) {
		for (int j = 0; j < N; j++) {
			if (features[i].histogram[j] < min_.histogram[j])
				min_.histogram[j] = features[i].histogram[j];
			if (features[i].histogram[j] > max_.histogram[j])
				max_.histogram[j] = features[i].histogram[j];
		}
	}

	// Normalize
	for (size_t i = 0; i < features.size(); i++) {
		for (int j = 0; j < N; j++) {
			if ((max_.histogram[j] - min_.histogram[j]) != 0) {
				features[i].histogram[j] = (features[i].histogram[j]
						- min_.histogram[j])
						/ (max_.histogram[j] - min_.histogram[j]);
			}
		}
	}

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::normalizeFeaturesWithCurrentMinMax(
		std::vector<FeatureT> & features) {
	int N = sizeof(min_.histogram) / sizeof(float);

	FeatureT min = min_, max = max_;

	// Find max and min values
	for (size_t i = 0; i < features.size(); i++) {
		for (int j = 0; j < N; j++) {
			if (features[i].histogram[j] < min.histogram[j])
				min.histogram[j] = features[i].histogram[j];
			if (features[i].histogram[j] > max.histogram[j])
				max.histogram[j] = features[i].histogram[j];
		}
	}

	// Normalize
	for (size_t i = 0; i < features.size(); i++) {
		for (int j = 0; j < N; j++) {
			if ((max_.histogram[j] - min_.histogram[j]) != 0) {
				features[i].histogram[j] = (features[i].histogram[j]
						- min.histogram[j])
						/ (max.histogram[j] - min.histogram[j]);
			}
		}
	}

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::vote() {

	pcl17::search::KdTree<FeatureT> feature_search;
	feature_search.setInputCloud(database_features_cloud_);

	for (size_t i = 0; i < features_.size(); i++) {
		std::vector<int> indices;
		std::vector<float> distances;
		feature_search.nearestKSearch(features_[i], num_neighbours_, indices,
				distances);

		if (debug_) {
			std::stringstream ss;
			ss << debug_folder_ << "Segment" << i << ".pcd";
			PointNormalCloud p(*scene_, *segment_indices_[i]);
			pcl17::io::savePCDFileASCII(ss.str(), p);
		}

		for (size_t j = 0; j < indices.size(); j++) {

			FeatureT closest_cluster = database_features_cloud_->at(indices[j]);
			for (std::map<std::string, pcl17::PointCloud<pcl17::PointXYZ> >::const_iterator it =
					database_[closest_cluster].begin();
					it != database_[closest_cluster].end(); it++) {

				std::string class_name = it->first;
				PointCloud model_centers = it->second;
				PointCloud model_centers_transformed;
				pcl17::PointCloud<pcl17::PointXYZI> model_centers_transformed_weighted;

				Eigen::Affine3f transform;
				transform.setIdentity();
				transform.translate(centroids_[i].getVector3fMap());
				pcl17::transformPointCloud(model_centers,
						model_centers_transformed, transform);

				pcl17::copyPointCloud(model_centers_transformed,
						model_centers_transformed_weighted);

				// TODO revise weighting function
				for (size_t k = 0;
						k < model_centers_transformed_weighted.size(); k++) {
					model_centers_transformed_weighted.points[k].intensity =
							exp(-(distances[j] * distances[j]))
									* (1.0 / model_centers.size());
					voted_segment_idx_[class_name].push_back(i);
				}

				if (debug_) {
					std::stringstream ss;
					ss << debug_folder_ << "Segment" << i << "_neighbour" << j
							<< "_" << class_name << "_votes.pcd";
					pcl17::io::savePCDFileASCII(ss.str(),
							model_centers_transformed_weighted);
				}

				votes_[class_name] += model_centers_transformed_weighted;
			}
		}

	}

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::vote_external(
		const std::string & matrix) {

	std::ifstream f4(matrix.c_str());

	pcl17::PointCloud<FeatureT> mat;
	std::vector<float> bias;

	int feature_size = sizeof(features_[0].histogram) / sizeof(float);

	std::string line;
	while (std::getline(f4, line))
	//while(f4)
	{

		// now we'll use a stringstream to separate the fields out of the line
		//std::stringstream ss( line );
		//std::string field;
		FeatureT ff;
		//int i=0;

		std::vector<std::string> st;
		boost::split(st, line, boost::is_any_of(","), boost::token_compress_on);

		//std::cerr << "Feature size " << feature_size << " Vector size " << st.size() <<  std::endl;

		bias.push_back(atof(st[0].c_str()));

		//while (std::getline( ss, field, ',' ))
		for (size_t i = 0; i < feature_size; i++) {
			// for each field we wish to convert it to a double
			// (since we require that the CSV contains nothing but floating-point values)

			//std::stringstream fs( field );
			//float v = 0;
			//fs >> v;
			//float v;
			//char tmp;
			//f4 >> v;
			//f4 >> tmp;
			//std::cerr << i << " " << st[i] << std::endl;
			ff.histogram[i] = atof(st[i + 1].c_str());
			//i++;
		}
		mat.points.push_back(ff);
		mat.width = mat.points.size();
		mat.height = 1;
	}

	//    string line;
	//    while (getline(f4, line))
	//    {
	//
	//      // now we'll use a stringstream to separate the fields out of the line
	//      stringstream ss(line);
	//      string field;
	//      FeatureT ff;
	//      int i = 0;
	//
	//      while (getline(ss, field, ','))
	//      {
	//        // for each field we wish to convert it to a double
	//        // (since we require that the CSV contains nothing but floating-point values)
	//        stringstream fs(field);
	//        float v = 0;
	//        fs >> v;
	//        ff.histogram[i] = v;
	//        i++;
	//      }
	//      mat.points.push_back(ff);
	//      mat.width = mat.points.size();
	//      mat.height = 1;
	//    }

	//pcl17::search::KdTree<FeatureT> feature_search;
	//feature_search.setInputCloud(database_features_cloud_);

	for (size_t i = 0; i < features_.size(); i++) {
		//std::vector<int> indices;
		//std::vector<float> distances;
		//feature_search.nearestKSearch(features_[i], num_neighbours_, indices, distances);

		Eigen::ArrayXf res = Eigen::ArrayXf::Zero(mat.points.size());

		for (size_t j = 0; j < mat.points.size(); j++) {
			res[j] += bias[j];
			for (size_t k = 0; k < feature_size; k++) {
				res[j] += mat[j].histogram[k] * features_[i].histogram[k];
			}
		}

		int cluster_idx;
		res.maxCoeff(&cluster_idx);
		//std::cerr << "Cluster number " << cluster_idx << std::endl;
		FeatureT ff;

		for (size_t k = 0; k < feature_size; k++) {
			ff.histogram[k] = 0;
		}

		ff.histogram[0] = cluster_idx;

		float prob = exp(res[cluster_idx]) / res.exp().sum();

		if (debug_) {
			std::stringstream ss;
			ss << debug_folder_ << "Segment" << i << ".pcd";
			PointNormalCloud p(*scene_, *segment_indices_[i]);
			pcl17::io::savePCDFileASCII(ss.str(), p);
		}

		//for (size_t j = 0; j < indices.size(); j++)
		//{

		//FeatureT closest_cluster = database_features_cloud_->at(indices[j]);
		for (std::map<std::string, pcl17::PointCloud<pcl17::PointXYZ> >::const_iterator it =
				database_[ff].begin(); it != database_[ff].end(); it++) {

			std::string class_name = it->first;
			PointCloud model_centers = it->second;
			PointCloud model_centers_transformed;
			pcl17::PointCloud<pcl17::PointXYZI> model_centers_transformed_weighted;

			Eigen::Affine3f transform;
			transform.setIdentity();
			transform.translate(centroids_[i].getVector3fMap());
			pcl17::transformPointCloud(model_centers, model_centers_transformed,
					transform);

			pcl17::copyPointCloud(model_centers_transformed,
					model_centers_transformed_weighted);

			// TODO revise weighting function
			for (size_t k = 0; k < model_centers_transformed_weighted.size();
					k++) {
				model_centers_transformed_weighted.points[k].intensity = prob
						* (1.0 / model_centers.size());
				voted_segment_idx_[class_name].push_back(i);
			}

			//          if (debug_)
			//          {
			//            std::stringstream ss;
			//            ss << debug_folder_ << "Segment" << i << "_neighbour" << j << "_" << class_name << "_votes.pcd";
			//            pcl17::io::savePCDFileASCII(ss.str(), model_centers_transformed_weighted);
			//          }

			votes_[class_name] += model_centers_transformed_weighted;
			//}
		}

	}

}

template<class PointT, class PointNormalT, class FeatureT>
Eigen::MatrixXf pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::projectVotesToGrid(
		const pcl17::PointCloud<pcl17::PointXYZI> & model_centers,
		int & grid_center_x, int & grid_center_y) {

	Eigen::MatrixXf grid;

	int image_x_width = (int) ((max_scene_bound_.x - min_scene_bound_.x)
			/ cell_size_);
	int image_y_width = (int) ((max_scene_bound_.y - min_scene_bound_.y)
			/ cell_size_);

	grid = Eigen::MatrixXf::Zero(image_x_width, image_y_width);

	for (size_t i = 0; i < model_centers.points.size(); i++) {
		int vote_x = (model_centers.points[i].x - min_scene_bound_.x)
				/ cell_size_;
		int vote_y = (model_centers.points[i].y - min_scene_bound_.y)
				/ cell_size_;
		if ((vote_x >= 0) && (vote_y >= 0) && (vote_x < image_x_width)
				&& (vote_y < image_y_width) && (model_centers.points[i].z >= 0))
			grid(vote_x, vote_y) += model_centers.points[i].intensity;
	}

	grid_center_x = -min_scene_bound_.x / cell_size_;
	grid_center_y = -min_scene_bound_.y / cell_size_;

	return grid;
}

template<class PointT, class PointNormalT, class FeatureT>
typename pcl17::PointCloud<PointT>::Ptr pcl17::PHVObjectClassifier<PointT,
		PointNormalT, FeatureT>::findLocalMaximaInGrid(Eigen::MatrixXf grid,
		float window_size) {

	PointCloudPtr local_maxima(new PointCloud);

	float max, min;
	max = grid.maxCoeff();
	min = grid.minCoeff();

	float threshold = min + (max - min) * local_maxima_threshold_;
	//float threshold = local_maxima_threshold_;

	//std::cerr << max << " " << min << " " << local_maxima_threshold_ << " " << threshold << std::endl;

	int window_size_pixels = window_size / cell_size_;

	if (window_size_pixels >= std::min(grid.cols() - 3, grid.rows() - 3)) {
		window_size_pixels = std::min(grid.cols() - 3, grid.rows() - 3);
	}

	// Make window_size_pixels even
	if (window_size_pixels % 2 == 0)
		window_size_pixels++;

	int side = window_size_pixels / 2;

	for (int i = 0; i < grid.rows(); i++) {
		for (int j = 0; j < grid.cols(); j++) {

			//float max;
			//Eigen::MatrixXf window = grid.block(i - side, j - side, window_size_pixels, window_size_pixels);
			//max = window.maxCoeff();

			//assert(window.cols() == window_size_pixels);
			//assert(window.rows() == window_size_pixels);

			// if max of the window is in its center then this point is local maxima
			if (/*(max == grid(i, j)) && (max > 0) &&*/(grid(i, j) > threshold)) {
				PointT point;
				point.x = i * cell_size_ + min_scene_bound_.x;
				point.y = j * cell_size_ + min_scene_bound_.y;
				point.z = 0;
				local_maxima->points.push_back(point);
			}
		}
	}

	local_maxima->width = local_maxima->points.size();
	local_maxima->height = 1;
	local_maxima->is_dense = true;

	return local_maxima;

}

template<class PointT, class PointNormalT, class FeatureT>
typename Eigen::ArrayXXi pcl17::PHVObjectClassifier<PointT, PointNormalT,
		FeatureT>::getLocalMaximaGrid(Eigen::MatrixXf & grid,
		float window_size) {

	Eigen::ArrayXXi local_max = Eigen::ArrayXXi::Zero(grid.rows(), grid.cols());

	float max, min;
	max = grid.maxCoeff();
	min = grid.minCoeff();

	//std::cerr << min << " " << max << std::endl;
	float threshold = min + (max - min) * local_maxima_threshold_;
	//float threshold = local_maxima_threshold_;

	int window_size_pixels = window_size / cell_size_;

	if (window_size_pixels >= std::min(grid.cols() - 3, grid.rows() - 3)) {
		window_size_pixels = std::min(grid.cols() - 3, grid.rows() - 3);
	}

	// Make window_size_pixels even
	if (window_size_pixels % 2 == 0)
		window_size_pixels++;

	int side = window_size_pixels / 2;

	for (int i = side; i < (grid.rows() - side); i++) {
		for (int j = side; j < (grid.cols() - side); j++) {

			float max;
			Eigen::MatrixXf window = grid.block(i - side, j - side,
					window_size_pixels, window_size_pixels);
			max = window.maxCoeff();

			assert(window.cols() == window_size_pixels);
			assert(window.rows() == window_size_pixels);

			// if max of the window is in its center then this point is local maxima
			if ((max == grid(i, j)) && (max > 0) && (max > threshold)) {
				local_max(i, j) = 1;
			}
		}
	}

	return local_max;

}

template<class PointT, class PointNormalT, class FeatureT>
vector<boost::shared_ptr<std::vector<int> > > pcl17::PHVObjectClassifier<PointT,
		PointNormalT, FeatureT>::findVotedSegments(
		typename pcl17::PointCloud<PointT>::Ptr local_maxima_,
		const string & class_name, float window_size) {

	vector<boost::shared_ptr<std::vector<int> > > voted_segments_;

	std::vector<std::set<int> > segment_combinations;

	for (size_t j = 0; j < local_maxima_->points.size(); j++) {
		PointT local_maxima = local_maxima_->points[j];
		std::vector<int> idx;

		for (size_t i = 0; i < votes_[class_name].points.size(); i++) {

			bool in_cell_x1 = votes_[class_name].points[i].x
					> (local_maxima.x - window_size / 2);
			bool in_cell_x2 = votes_[class_name].points[i].x
					< (local_maxima.x + window_size / 2);
			bool in_cell_y1 = votes_[class_name].points[i].y
					> (local_maxima.y - window_size / 2);
			bool in_cell_y2 = votes_[class_name].points[i].y
					< (local_maxima.y + window_size / 2);

			if (in_cell_x1 && in_cell_x2 && in_cell_y1 && in_cell_y2) {
				idx.push_back(i);
			}
		}

		std::set<int> segment_idx;

		for (size_t i = 0; i < idx.size(); i++) {
			segment_idx.insert(voted_segment_idx_[class_name][idx[i]]);
		}

		segment_combinations.push_back(segment_idx);

	}

	std::unique(segment_combinations.begin(), segment_combinations.end());

	for (size_t i = 0; i < segment_combinations.size(); i++) {
		//PointNormalCloudPtr cloud(new PointNormalCloud);
		boost::shared_ptr<std::vector<int> > cloud_idx(new std::vector<int>);

		for (std::set<int>::iterator it = segment_combinations[i].begin();
				it != segment_combinations[i].end(); it++) {
			//PointNormalCloud segment(*scene_, *segment_indices_[*it]);
			cloud_idx->insert(cloud_idx->begin(),
					segment_indices_[*it]->begin(),
					segment_indices_[*it]->end());

			//*cloud += segment;
		}

		voted_segments_.push_back(cloud_idx);

	}

	if (debug_) {

		for (size_t i = 0; i < voted_segments_.size(); i++) {
			PointNormalCloud seg(*scene_, *voted_segments_[i]);
			std::stringstream ss;
			ss << debug_folder_ << class_name << "_segment_" << i << ".pcd";
			std::cerr << "Writing to file " << ss.str() << std::endl;
			pcl17::io::savePCDFileASCII(ss.str(), seg);
		}
	}

	return voted_segments_;

}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::fitModelsWithRansac(
		vector<boost::shared_ptr<std::vector<int> > > & voted_segments_,
		const string class_name,
		RandomSampleConsensusSimple<PointNormalT> & ransac,
		vector<PointNormalCloudPtr> & result_, vector<float> & scores_) {

	for (size_t i = 0; i < voted_segments_.size(); i++) {

		ransac.setSceneSegment(voted_segments_[i]);

		ransac.computeModel();

		float score = ransac.getBestScore();

		std::cerr << "Score " << score << std::endl;

		if (score > ransac_result_threshold_[class_name]) {
			Eigen::VectorXf model_coefs = ransac.getBestModelCoeficients();

			Eigen::Affine3f transform;
			transform.setIdentity();
			transform.translate(
					Eigen::Vector3f(model_coefs[0], model_coefs[1], 0));
			transform.rotate(
					Eigen::AngleAxisf(model_coefs[2],
							Eigen::Vector3f(0, 0, 1)));

			PointNormalCloudPtr full_model_transformed(new PointNormalCloud);
			pcl17::transformPointCloudWithNormals(*ransac.getModel(),
					*full_model_transformed, transform);

			result_.push_back(full_model_transformed);
			scores_.push_back(1 - score);

		}
	}
}

template<class PointT, class PointNormalT, class FeatureT>
void pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::generateVisibilityScore(
		vector<PointNormalCloudPtr> & result_, vector<float> & scores_) {

	pcl17::octree::OctreePointCloudSearch<PointNormalT> octree(
			subsampling_resolution_ * 2.5f);
	octree.setInputCloud(scene_);
	octree.addPointsFromInputCloud();

	for (size_t i = 0; i < result_.size(); i++) {
		int free = 0, occupied = 0, occluded = 0;
		for (size_t j = 0; j < result_[i]->points.size(); j++) {
			PointNormalT point = result_[i]->points[j];

			if (octree.isVoxelOccupiedAtPoint(point)) {
				occupied++;

				continue;
			}

			Eigen::Vector3f sensor_orig = scene_->sensor_origin_.head(3);
			Eigen::Vector3f look_at = point.getVector3fMap() - sensor_orig;

			std::vector<int> indices;
			octree.getIntersectedVoxelIndices(sensor_orig, look_at, indices);

			bool is_occluded = false;
			if (indices.size() > 0) {
				for (size_t k = 0; k < indices.size(); k++) {
					Eigen::Vector3f ray =
							scene_->points[indices[k]].getVector3fMap()
									- sensor_orig;

					if (ray.norm() < look_at.norm()) {
						is_occluded = true;
					}

				}
			}

			if (is_occluded) {
				occluded++;
				continue;
			}

			free++;

		}

		scores_[i] = 1
				- ((float) 2 * occupied + occluded)
						/ (2 * occupied + occluded + free);
		//std::cerr << "Score " << occupied << " " << occluded << " " << free << " " << score[i] << std::endl;

	}

}

template<class PointT, class PointNormalT, class FeatureT>
bool pcl17::PHVObjectClassifier<PointT, PointNormalT, FeatureT>::intersectXY(
		const pcl17::PointCloud<PointNormalT> & cloud1,
		const pcl17::PointCloud<PointNormalT> & cloud2) {

	PointNormalT min1, max1, min2, max2;
	pcl17::getMinMax3D<PointNormalT>(cloud1, min1, max1);
	pcl17::getMinMax3D<PointNormalT>(cloud2, min2, max2);

	bool intersectX, intersectY;
	if (min1.x < min2.x)
		intersectX = max1.x > min2.x;
	else if (min1.x > min2.x)
		intersectX = max2.x > min1.x;
	else
		// min1.x == min2.x
		intersectX = true;

	if (min1.y < min2.y)
		intersectY = max1.y > min2.y;
	else if (min1.y > min2.y)
		intersectY = max2.y > min1.y;
	else
		// min1.y == min2.y
		intersectY = true;

	return intersectX && intersectY;

}

template<class PointT, class PointNormalT, class FeatureT>
vector<typename pcl17::PointCloud<PointNormalT>::Ptr> pcl17::PHVObjectClassifier<
		PointT, PointNormalT, FeatureT>::removeIntersecting(
		vector<typename pcl17::PointCloud<PointNormalT>::Ptr> & result_,
		vector<float> & scores_, vector<float> * selected_scores) {

	vector<PointNormalCloudPtr> no_intersect_result;

	if (result_.size() == 0)
		return no_intersect_result;

	for (size_t i = 0; i < result_.size(); i++) {
		bool best = true;
		for (size_t j = 0; j < result_.size(); j++) {
			if (intersectXY(*result_[i], *result_[j])) {
				if (scores_[i] > scores_[j])
					best = false;
			}

		}
		if (best) {
			if (selected_scores) {
				selected_scores->push_back(scores_[i]);
			}
			no_intersect_result.push_back(result_[i]);
		}
	}

	return no_intersect_result;

}

#endif /* PHVOBJECTCLASSIFIER_HPP_ */
