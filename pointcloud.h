#pragma once

struct PointCloud_Point
{
	size_t id;
	double  x, y;
	PointCloud_Point() :id(0), x(0.0), y(0.0) {};
	PointCloud_Point(const size_t id, const double x, const double y) : id(id), x(x), y(y) {};
	PointCloud_Point(const PointCloud_Point& pt) :id(pt.id), x(pt.x), y(pt.y) {};
};

struct PointCloud
{
	
	std::vector<PointCloud_Point> pts;

	inline double kdtree_distance(const double *p1, const size_t idx_p2, size_t /*size*/) const
	{
		const double d0 = p1[0] - pts[idx_p2].y;
		const double d1 = p1[1] - pts[idx_p2].x;
		return d0*d0 + d1*d1 ;
	}
	// ポイント数を返す必要があります。
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// 次元数に対応する成分を返します。
	inline double kdtree_get_pt(const size_t idx, int dim) const
	{
		if(dim == 0)
			return pts[idx].x; 
		else
			return pts[idx].y;
	}

	// オプションのバウンディングボックス計算：標準のbbox計算ループをデフォルトにするにはfalseを返します。
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};
