#include "testClass.h"


void call_vertex_position(Eigen::MatrixXd V) {
	std::cout << V << std::endl;
}

Eigen::MatrixXd move_mash(Eigen::MatrixXd &V, Eigen::RowVector3d m)
{	
	std::cout << V.rows() << std::endl;
	for (int i = 0; i < V.rows(); i++) {
		
		//std::cout << i << std::endl;
		//std::cout << V.row(i) << "\n" << m << std::endl;
		V.row(i) += m;
	}
	
	return (V);
}
