#include "kdtree.h"
#include <iostream>
// コンストラクタ
KDTree::KDTree() 
{
	build_index({});
}
// コンストラクタ
KDTree::KDTree(const vector<PointCloud_Point>& pts)
{	
	build_index(pts);

}
// コンストラクタ
KDTree::~KDTree() = default;
// kd-tree インデックスの構築
void KDTree::build_index(const vector<PointCloud_Point>& pts)
{
	// 座標データを退避
	// データのクリア
	m_id_map.clear();
	m_point_cloud.pts.clear();
	// kd-treeの初期化
	m_kdtree.reset(new my_kd_tree_t(2, m_point_cloud, KDTreeSingleIndexAdaptorParams()));

	if (pts.size() > 0)
	{
		//座標データの追加し直し
		for (const PointCloud_Point& it : pts)
		{
			m_id_map[it.id] = m_point_cloud.pts.size();
			m_point_cloud.pts.push_back(it);
		}
		//kd-treeの再構築
		m_kdtree->addPoints(0, m_point_cloud.pts.size() - 1);
	}
}
//座標データの取得
PointCloud_Point & KDTree::get_point(const size_t id)  
{

	const auto iter = m_id_map.find(id);
#ifdef DEBUG
	if (iter != m_id_map.end())	cout << "ID: %d does not exists", id); << endl;
#endif // DEBUG
	size_t i = iter->second;
#ifdef DEBUG
	if (i < m_point_cloud.pts.size())	cout << "Index out bounds, index:", i << endl;
#endif // DEBUG
	return  m_point_cloud.pts[i];
}
//座標データの追加
size_t KDTree::add_point(const size_t id, const double x, const double y)
{
	const size_t last = m_point_cloud.pts.size();

	m_point_cloud.pts.emplace_back(id, x, y);	
	m_kdtree->addPoints(last, last);
	m_id_map[id] = last;
	return last;
}
// //半径距離による検索
vector<PointCloud_Point> KDTree::radius_search(const PointCloud_Point &pt, const double distance)
{
	// パラメータの設定
	const double query_pt[2]{ pt.x, pt.y };
	nanoflann::SearchParams params;
	params.sorted = false;
	// 検索結果出力パラメータの設置
	vector<pair<size_t, double>> indices_dists;
	RadiusResultSet<double, size_t> resultSet(distance, indices_dists);
	// 検索
	m_kdtree->findNeighbors(resultSet, query_pt, params);

	// 座標リストに変換
	int i = 0;
	vector<PointCloud_Point> result(resultSet.size());
	for (const pair<size_t, double>& it : resultSet.m_indices_dists)
	{
		result[i++] = m_point_cloud.pts[it.first];
	}
	return move(result);

}
