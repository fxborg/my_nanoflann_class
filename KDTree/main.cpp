#include <fstream>
#include <string>
#include <vector>

#include <ctime>
#include <cstdlib>
#include <iostream>
#include "kdtree.h"
using namespace std;

int main()
{
	// KDTree 
	KDTree tree;

	// 座標データの読み込み
	string src_path = "data.txt";
	ifstream fin(src_path);
	if (fin.bad()) {
		cout << "File Open Error:" << src_path << std::endl;
		return 1;
	}
	std::string str;
	size_t i=0;
	while (std::getline(fin, str))
	{
		double x,y;
		int res = sscanf_s(str.c_str(), "%lf\t%lf", &x, &y);

		if (res != 2)continue;
		// データをツリーに格納
		tree.add_point(++i, x, y);
	}
	fin.close();
	// 半径 座標の検索
	double dist =20.0;

	cout << " (60,130)から半径"<< dist<<"の距離以内にある座標" << endl;

	PointCloud_Point pt{ ++i,60.0,130 };
	vector<PointCloud_Point> rad_result = tree.radius_search(pt,dist*dist);
	// 結果出力
	cout << " x  y" << endl;
	cout << "---------" << endl;
	for (const PointCloud_Point& it : rad_result)
	{
		cout <<" "<< it.x << " " << it.y << endl;
	}
	cout << endl;

	system("pause");
	return 0;
}