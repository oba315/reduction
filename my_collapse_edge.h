#pragma once

#ifndef MY_COLLAPSE_EDGE_H
#define MY_COLLAPSE_EDGE_H
#include <igl/igl_inline.h>
#include <Eigen/Core>
#include <vector>
#include <set>

// IGL_INLINE������Ɩ������̊O���Q�ƂɂȂ�D�C�����C���������ق��������̂��H

namespace mycollapse
{
    // Assumes (V,F) is a closed manifold mesh (except for previously collapsed
    // faces which should be set to: 
    // [IGL_COLLAPSE_EDGE_NULL IGL_COLLAPSE_EDGE_NULL IGL_COLLAPSE_EDGE_NULL].
    // Collapses exactly two faces and exactly 3 edges from E (e and one side of
    // each face gets collapsed to the other). This is implemented in a way that
    // it can be repeatedly called until satisfaction and then the garbage in F
    // can be collected by removing NULL faces.
    //
    // Inputs:
    //   e  index into E of edge to try to collapse. E(e,:) = [s d] or [d s] so
    //     that s<d, then d is collapsed to s.
    ///  p  dim list of vertex position where to place merged vertex
    // Inputs/Outputs:
    //   V  #V by dim list of vertex positions, lesser index of E(e,:) will be set
    //     to midpoint of edge.
    //   F  #F by 3 list of face indices into V.
    //   E  #E by 2 list of edge indices into V.
    //   EMAP #F*3 list of indices into E, mapping each directed edge to unique
    //     unique edge in E
    //   EF  #E by 2 list of edge flaps, EF(e,0)=f means e=(i-->j) is the edge of
    //     F(f,:) opposite the vth corner, where EI(e,0)=v. Similarly EF(e,1) "
    //     e=(j->i)
    //   EI  #E by 2 list of edge flap corners (see above).
    //   e1  index into E of edge collpased on left
    //   e2  index into E of edge collpased on right
    //   f1  index into F of face collpased on left
    //   f2  index into F of face collpased on right
    // Returns true if edge was collapsed
#define IGL_COLLAPSE_EDGE_NULL 0
    bool collapse_edge(
        const int e,
        const Eigen::RowVectorXd& p,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        Eigen::MatrixXi& E,
        Eigen::VectorXi& EMAP,
        Eigen::MatrixXi& EF,
        Eigen::MatrixXi& EI,
        int& e1,
        int& e2,
        int& f1,
        int& f2);

    
     bool collapse_edge(
        const int e,
        const Eigen::RowVectorXd& p,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        Eigen::MatrixXi& E,
        Eigen::VectorXi& EMAP,
        Eigen::MatrixXi& EF,
        Eigen::MatrixXi& EI);
    // Collapse least-cost edge from a priority queue and update queue 
    //
    // Inputs/Outputs:
    //   cost_and_placement  function computing cost of collapsing an edge and 3d
    //     position where it should be placed:
    //     cost_and_placement(V,F,E,EMAP,EF,EI,cost,placement);
    //     **If the edges is collapsed** then this function will be called on all
    //     edges of all faces previously incident on the endpoints of the
    //     collapsed edge.
    //   Q  queue containing pairs of costs and edge indices
    //   Qit  list of iterators so that Qit[e] --> iterator of edge e in Q
    //   C  #E by dim list of stored placements
    bool collapse_edge(
        const std::function<void(
            const int,
            const Eigen::MatrixXd&,
            const Eigen::MatrixXi&,
            const Eigen::MatrixXi&,
            const Eigen::VectorXi&,
            const Eigen::MatrixXi&,
            const Eigen::MatrixXi&,
            double&,
            Eigen::RowVectorXd&)>& cost_and_placement,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        Eigen::MatrixXi& E,
        Eigen::VectorXi& EMAP,
        Eigen::MatrixXi& EF,
        Eigen::MatrixXi& EI,
        std::set<std::pair<double, int> >& Q,
        std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
        Eigen::MatrixXd& C,
        Eigen::VectorXd& weight);
    // Inputs:
    //   pre_collapse  callback called with index of edge whose collapse is about
    //     to be attempted. This function should return whether to **proceed**
    //     with the collapse: returning true means "yes, try to collapse",
    //     returning false means "No, consider this edge 'uncollapsable', behave
    //     as if collapse_edge(e) returned false.
    //   post_collapse  callback called with index of edge whose collapse was
    //     just attempted and a flag revealing whether this was successful.
    bool collapse_edge(
        const std::function<void(
            const int,
            const Eigen::MatrixXd&,
            const Eigen::MatrixXi&,
            const Eigen::MatrixXi&,
            const Eigen::VectorXi&,
            const Eigen::MatrixXi&,
            const Eigen::MatrixXi&,
            double&,
            Eigen::RowVectorXd&)>& cost_and_placement,
        const std::function<bool(
            const Eigen::MatrixXd&,/*V*/
            const Eigen::MatrixXi&,/*F*/
            const Eigen::MatrixXi&,/*E*/
            const Eigen::VectorXi&,/*EMAP*/
            const Eigen::MatrixXi&,/*EF*/
            const Eigen::MatrixXi&,/*EI*/
            const std::set<std::pair<double, int> >&,/*Q*/
            const std::vector<std::set<std::pair<double, int> >::iterator >&,/*Qit*/
            const Eigen::MatrixXd&,/*C*/
            const int                                                        /*e*/
            )>& pre_collapse,
        const std::function<void(
            const Eigen::MatrixXd&,   /*V*/
            const Eigen::MatrixXi&,   /*F*/
            const Eigen::MatrixXi&,   /*E*/
            const Eigen::VectorXi&,/*EMAP*/
            const Eigen::MatrixXi&,  /*EF*/
            const Eigen::MatrixXi&,  /*EI*/
            const std::set<std::pair<double, int> >&,   /*Q*/
            const std::vector<std::set<std::pair<double, int> >::iterator >&, /*Qit*/
            const Eigen::MatrixXd&,   /*C*/
            const int,   /*e*/
            const int,  /*e1*/
            const int,  /*e2*/
            const int,  /*f1*/
            const int,  /*f2*/
            const bool                                                  /*collapsed*/
            )>& post_collapse,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        Eigen::MatrixXi& E,
        Eigen::VectorXi& EMAP,
        Eigen::MatrixXi& EF,
        Eigen::MatrixXi& EI,
        std::set<std::pair<double, int> >& Q,
        std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
        Eigen::MatrixXd& C,
        Eigen::VectorXd& weight);

    bool collapse_edge(
        const std::function<void(
            const int,
            const Eigen::MatrixXd&,
            const Eigen::MatrixXi&,
            const Eigen::MatrixXi&,
            const Eigen::VectorXi&,
            const Eigen::MatrixXi&,
            const Eigen::MatrixXi&,
            double&,
            Eigen::RowVectorXd&)>& cost_and_placement,
        const std::function<bool(
            const Eigen::MatrixXd&,/*V*/
            const Eigen::MatrixXi&,/*F*/
            const Eigen::MatrixXi&,/*E*/
            const Eigen::VectorXi&,/*EMAP*/
            const Eigen::MatrixXi&,/*EF*/
            const Eigen::MatrixXi&,/*EI*/
            const std::set<std::pair<double, int> >&,/*Q*/
            const std::vector<std::set<std::pair<double, int> >::iterator >&,/*Qit*/
            const Eigen::MatrixXd&,/*C*/
            const int                                                        /*e*/
            )>& pre_collapse,
        const std::function<void(
            const Eigen::MatrixXd&,   /*V*/
            const Eigen::MatrixXi&,   /*F*/
            const Eigen::MatrixXi&,   /*E*/
            const Eigen::VectorXi&,/*EMAP*/
            const Eigen::MatrixXi&,  /*EF*/
            const Eigen::MatrixXi&,  /*EI*/
            const std::set<std::pair<double, int> >&,   /*Q*/
            const std::vector<std::set<std::pair<double, int> >::iterator >&, /*Qit*/
            const Eigen::MatrixXd&,   /*C*/
            const int,   /*e*/
            const int,  /*e1*/
            const int,  /*e2*/
            const int,  /*f1*/
            const int,  /*f2*/
            const bool                                                  /*collapsed*/
            )>& post_collapse,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        Eigen::MatrixXi& E,
        Eigen::VectorXi& EMAP,
        Eigen::MatrixXi& EF,
        Eigen::MatrixXi& EI,
        std::set<std::pair<double, int> >& Q,
        std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
        Eigen::MatrixXd& C,
        int& e,
        int& e1,
        int& e2,
        int& f1,
        int& f2,
        Eigen::VectorXd& weight);

    
    void shortest_edge_and_midpoint(
        const int e,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& /*F*/,
        const Eigen::MatrixXi& E,
        const Eigen::VectorXi& /*EMAP*/,
        const Eigen::MatrixXi& EF/*EF*/,
        const Eigen::MatrixXi& /*EI*/,
        double& cost,
        Eigen::RowVectorXd& p,
        Eigen::VectorXd& weight);
    


}



//#ifndef IGL_STATIC_LIBRARY
//#  include "my_collapse_edge.cpp"
//#endif
#endif