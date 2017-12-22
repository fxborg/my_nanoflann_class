#pragma once

#include <memory>
#include <unordered_map>
#include "pointcloud.h"
#include "nanoflann.hpp"

using namespace std;
using namespace nanoflann;

// kd-treeのインデックス型定義:
typedef KDTreeSingleIndexDynamicAdaptor<
	L2_Simple_Adaptor<double, PointCloud >,
	PointCloud, 2
> my_kd_tree_t;

// KDTree クラス
class KDTree {
public:
	
	KDTree();
	KDTree(const vector<PointCloud_Point> & pts);
	~KDTree();

	void build_index(const vector<PointCloud_Point> & pts);							// kd-tree インデックスの構築
	PointCloud_Point & get_point(const size_t id) ;	//座標データの取得
	size_t add_point(const size_t id, const double x, const double y); //座標データの追加

	vector<PointCloud_Point>  radius_search(	//半径距離による検索
		const PointCloud_Point & pt,
		const double distance
	); 



private:
	PointCloud m_point_cloud; //座標データ群
	unordered_map<size_t, size_t> m_id_map; //インデックス
	unique_ptr<my_kd_tree_t> m_kdtree; //kd-treeのポインタ 
};


