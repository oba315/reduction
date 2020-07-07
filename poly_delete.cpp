#include "poly_delete.h"
#include <vector>
#include "matrix_tools.h"
#include <time.h>

namespace poldel {

	//マスクに応じて面を削除
	void poly_delete(
		Eigen::MatrixXd& V,
		Eigen::MatrixXi& F,
		Eigen::MatrixXd& C,
		Eigen::VectorXi& mask, //1の面を削除
		bool debug
	)
	{
		std::cout << "面を削除します．\ntime:start" << std::endl;
		clock_t start = clock();
		int bf = F.rows();
		
		std::vector<int> fi;
		
		for (int i = 0; i < mask.rows(); i++) {
			if (mask[i] == 1) {
				fi.push_back(i);
			}
		}
		std::cout << fi.size() << std::endl;

		Eigen::MatrixXi SVJ;
		remove_row(F, SVJ, fi);
		std::cout << "time:" << double(clock() - start) << "\n";
		Eigen::MatrixXi SVJC;
		remove_row(C, SVJC, fi);
		std::cout << "time:" << double(clock() - start) << "\n";
		std::cout << fi.size() << "個の面を削除しました．" << bf << "->" << F.rows() << std::endl;

	}

}
