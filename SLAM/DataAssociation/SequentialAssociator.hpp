#ifndef SEQUENTIAL_ASSOCIATOR_HPP
#define SEQUENTIAL_ASSOCIATOR_HPP

#include <iostream>
#include <vector>
#include <boost/function.hpp>
#include <cmath>

#include <Eigen/Dense>

#include "../Base/DMDebug.h"

IMPORT_DEBUG_LOG()

#define ITERATOR_POS(IT,BEG,N) do { (IT)=(BEG); std::advance((IT),(N)); }while(0)

template<typename T1,typename T2, typename D>
class SequentialAssociator
{
public:
  typedef T1 Value1;
  typedef T2 Value2;
  typedef D Distance;
  typedef typename boost::function<D(T1, T2)> Metric;
  typedef std::vector<T1> ValueVector1;
  typedef std::vector<T2> ValueVector2;
  typedef std::vector<std::pair<int,int> > AssociationVector;
  typedef Eigen::Matrix<Distance,Eigen::Dynamic,Eigen::Dynamic> Matrix;

  SequentialAssociator(Metric metr,bool _multiple = false)
   : metric(metr), distances(), multipleAssociationOn(_multiple)
  { }

  SequentialAssociator(Metric metr,int nmax,bool _multiple = false)
   : metric(metr), distances(nmax,nmax), multipleAssociationOn(_multiple) 
  { 
    assert(nmax >= 0);
  }
  
  const Matrix& getDistanceMatrix() const
  {
    return distances;
  }

  void associate(const ValueVector1& v1,const ValueVector2& v2,
                 AssociationVector& associations);

  template <typename It1,typename It2>
  void associate(It1 beg1,It1 end1,It2 beg2,It2 end2,
                 std::vector<std::pair<It1,It2> >& associations);
  
protected:
  Metric metric;
  Matrix distances;
  bool multipleAssociationOn;
};

template <typename T1,typename T2,typename D>
void SequentialAssociator<T1,T2,D>::associate(const ValueVector1& v1,const ValueVector2& v2,
  AssociationVector& associations)
{
  if (v1.empty() || v2.empty()) {
    std::cerr << __FILE__ << "," << __LINE__ << ": associating an empty set!\n";
    return;
  }
  int rowNum = v1.size();
  int colNum = v2.size();
  // Resizes the distances matrix if it is not large enough
  if (distances.rows() < rowNum && distances.cols() < colNum) {
    distances.resize(rowNum,colNum);
  }
  else if (distances.rows() < rowNum) {
    distances.resize(rowNum,Eigen::NoChange);
  }
  else if (distances.cols() < colNum) {
    distances.resize(Eigen::NoChange,colNum);
  }
  // Fills the first row and first column with one-to-one distance
  for (int r = 0; r < rowNum; ++r) {
    distances(r,0) = metric(v1[r],v2[0]);
  }
  for (int c = 1; c < colNum; ++c) {
    distances(0,c) = metric(v1[0],v2[c]);
  }
  // Fills the remaining cells
  for (int r = 1; r < rowNum; ++r) {
    for (int c = 1; c < colNum; ++c) {
      distances(r,c) = metric(v1[r],v2[c]) + 
        std::min( distances(r-1,c-1), std::min(distances(r,c-1),distances(r-1,c)) );
    }
  }
//  std::cout << "Matrix\n" << distances << "\n";
  // Visists the cells to build associations
  int r = rowNum-1;
  int c = colNum-1;
  int rprev = r;
  int cprev = c;
  int rbest = r;
  int cbest = c;
  Distance minDist;
  Distance minBest = metric(v1[r],v2[c]);
  while (r > 0 || c > 0) {
    // If multipleAssociationOn is true, all the values on minimum path are 
    // inserted as associations.
    // Otherwise, it inserts exactly one value per each row or column in the 
    // path from (rowNum-1,colNum-1) to (0,0). 
//     if (multipleAssociationOn) {
//       if (r != rprev || c != cprev) {
//         associations.push_back(std::make_pair(r,c));
//       }
//     }
//     else {
//       if (r != rprev && c != cprev) {
// //        std::cout << "  rbest " << rbest << ", cbest " << cbest << ", values " << v1[rbest] << ":" << v2[cbest] << "\n";
//         associations.push_back(std::make_pair(rbest,cbest));
//         // Re-initialize rbest, cbest, minBest
//         rbest = r;
//         cbest = c;
//         minBest = metric(v1[r],v2[c]); 
//       }
//       else if (metric(v1[r],v2[c]) < minBest) {
//         rbest = r;
//         cbest = c;
//         minBest = metric(v1[r],v2[c]);
//       }
//     }
   DLOG() << "row " << r << ", col " << c <<"; value v1 " << v1[r] << " v2 " << v2[c] << ", cost " << distances(r,c) 
     << ", values " << v1[r] << ":" << v2[c] << std::endl;
    // Updates the row/col indices r and c toward the minimum between 
    // cells (r-1,c), (r,c-1) and (r-1,c-1)
    rprev = r;
    cprev = c;
	if (rprev == 0 && cprev > 0) {
      r = rprev;
	  c = cprev - 1;
	}
	else if (rprev > 0 && cprev == 0) {
	  r = rprev - 1;
	  c = cprev;
	}
	else if (rprev > 0 && cprev > 0) {
	  minDist = distances(rprev,cprev);
	  if (distances(rprev-1,cprev) <= minDist) {
        r = rprev - 1;
		c = cprev;
	  }
	  else if (distances(rprev,cprev-1) <= minDist) {
		r = rprev;
		c = cprev - 1;
	  }
	  else if (distances(rprev-1,cprev-1) <= minDist) {
		r = rprev - 1;
		c = cprev - 1;
	  }
	  else {
		assert(0);
	  }
	}
	
	if (multipleAssociationOn) {
	  if (r != rprev || c != cprev) {
		associations.push_back(std::make_pair(rprev,cprev));
	  }
	}
	else {
	  if (r != rprev && c != cprev) {
		associations.push_back(std::make_pair(rprev,cprev));
	  }
	}
//     minDist = distances(r,c);
// //    std::cout << " current minDist " << minDist << std::endl;
//     if (rprev > 0 && distances(rprev-1,cprev) <= minDist) {
//       r = rprev - 1;
//       minDist = distances(rprev-1,cprev);
//     }
//     else if (rprev == 0 && cprev > 0) {
//       c = cprev - 1;
//     }
// //    (rprev > 0) && std::cout << " row up " << (rprev-1) << "," << cprev << " cost " << distances(rprev-1,cprev) << ", new cost" << minDist << std::endl;
//     if (cprev > 0 && 
//         (distances(rprev,cprev-1) < minDist ||
//          (distances(rprev,cprev-1) == minDist && 
//           ((r == rprev && c == cprev) || metric(v1[rprev],v2[cprev-1]) <= metric(v1[r],v2[c]))
//          )
//         )
//        )
//     {
//       c = cprev - 1;
//       minDist = distances(rprev,cprev-1);
//     }
//     else if (cprev == 0 && rprev > 0) {
//       r = rprev - 1;
//     }
// //    (cprev > 0) && std::cout << " col up " << rprev << "," << (cprev-1) << " cost " << distances(rprev,cprev-1) << ", new cost" << minDist << std::endl;
//     if (rprev > 0 && cprev > 0 && 
//         (distances(rprev-1,cprev-1) < minDist ||
//          (distances(rprev-1,cprev-1) == minDist && 
//           ((r == rprev && c == cprev) || metric(v1[rprev-1],v2[cprev-1]) <= metric(v1[r],v2[c]))
//          )
//         )
//        )
//     {
//       r = rprev - 1;
//       c = cprev - 1;
//       minDist = distances(rprev-1,cprev-1); 
//     }
//    (rprev > 0 && cprev > 0) && std::cout << " diag " << (rprev-1) << "," << (cprev-1) << " cost " << distances(rprev-1,cprev-1) << ", new cost" << minDist << std::endl;
//    std::cout << " *** move from " << rprev << "," << cprev << " to " << r << "," << c << std::endl;
  } 
  assert(r == 0 && c == 0);
  
//  std::cout << "r " << r << " c " << c << " v1 " << v1[r] << " v2 " << v2[c]
//     << "; rprev " << rprev << ", cprev " << cprev << " v1 " << v1[rprev] << " v2 " << v2[cprev]
//     << "; rbest " << rbest << ", cbest " << cbest << " v1 " << v1[rbest] << " v2 " << v2[cbest]
//     << "; best cost " << distances(rbest,cbest) << std::endl;
//   if (multipleAssociationOn) {
// 	DLOG() << "row " << r << ", col " << c << ", values " << v1[r] << ":" << v2[c] << " END" << std::endl;
//     associations.push_back(std::make_pair(r,c));
//   }
//   else {
//     DLOG() << "row " << rbest << ", col " << cbest << ", cost " << distances(rbest,cbest) << std::endl;
// //    associations.push_back(std::make_pair(rbest,cbest));
//      if (r != rprev && c != cprev) {
// //        std::cout << "  rbest " << rbest << ", cbest " << cbest << ", values " << v1[rbest] << ":" << v2[cbest] << "\n";
//         associations.push_back(std::make_pair(rbest,cbest));
//         // Re-initialize rbest, cbest, minBest
//         rbest = r;
//         cbest = c;
//         minBest = metric(v1[r],v2[c]); 
//       }
//       else if (metric(v1[r],v2[c]) < minBest) {
//         rbest = r;
//         cbest = c;
//         minBest = metric(v1[r],v2[c]);
//       }
//      associations.push_back(std::make_pair(rbest,cbest));
//   }
//   std::reverse(associations.begin(),associations.end());
//   DLOG() << "association vector contains " << associations.size() << " associations" << std::endl;
}

// --------------------------------------------------------
// The iterator-based version of associate()
// --------------------------------------------------------

template <typename T1,typename T2,typename D>
template <typename It1,typename It2>
void SequentialAssociator<T1,T2,D>::associate(It1 beg1,It1 end1,It2 beg2,It2 end2,
                                              std::vector<std::pair<It1,It2> >& associations)
{
  It1 it1;
  It2 it2;
  // Checks if the intervals are empty
  if (beg1 == end1 || beg2 == end2) {
    std::cerr << __FILE__ << "," << __LINE__ << ": associating an empty set!\n";
    return;
  }
  int rowNum = std::distance(beg1,end1);
  int colNum = std::distance(beg2,end2);
  // Resizes the distances matrix if it is not large enough
  if (distances.rows() < rowNum && distances.cols() < colNum) {
    distances.resize(rowNum,colNum);
  }
  else if (distances.rows() < rowNum) {
    distances.resize(rowNum,Eigen::NoChange);
  }
  else if (distances.cols() < colNum) {
    distances.resize(Eigen::NoChange,colNum);
  }
  // Fills the first row and first column with one-to-one distance
  it1 = beg1;
  for (int r = 0; r < rowNum && it1 != end1; ++r, ++it1) {
    distances(r,0) = metric(*it1,*beg2);
  }
  it2 = beg2;
  for (int c = 0; c < colNum && it2 != end2; ++c, ++it2) {
    distances(0,c) = metric(*beg1,*it2);
  }
  // Fills the remaining cells
  for (int r = 1; r < rowNum; ++r) {
    for (int c = 1; c < colNum; ++c) {
      ITERATOR_POS(it1,beg1,r);
      ITERATOR_POS(it2,beg2,c);
      distances(r,c) = metric(*it1,*it2) + 
        std::min( distances(r-1,c-1), std::min(distances(r,c-1),distances(r-1,c)) );
    }
  }
  // Visists the cells to build associations
  int r = rowNum-1;
  int c = colNum-1;
  int rprev = r;
  int cprev = c;
  int rbest = r;
  int cbest = c;
  ITERATOR_POS(it1,beg1,r);
  ITERATOR_POS(it2,beg2,c);
  Distance minDist;
  Distance minBest = metric(*it1,*it2);
  while (r > 0 || c > 0) {
    // If multipleAssociationOn is true, all the values on minimum path are 
    // inserted as associations.
    // Otherwise, it inserts exactly one value per each row or column in the 
    // path from (rowNum-1,colNum-1) to (0,0). 
    if (multipleAssociationOn) {
      if (r != rprev || c != cprev) {
        associations.push_back(std::make_pair(it1,it2));
      }
    }
    else {
      if (r != rprev && c != cprev) {
        ITERATOR_POS(it1,beg1,rbest);
        ITERATOR_POS(it2,beg2,cbest);
        associations.push_back(std::make_pair(it1,it2));
        // Re-initialize rbest, cbest, minBest
        rbest = r;
        cbest = c;
        ITERATOR_POS(it1,beg1,r);
        ITERATOR_POS(it2,beg2,c);
        minBest = metric(*it1,*it2); 
      }
      else if (metric(*it1,*it2) < minBest) {
        rbest = r;
        cbest = c;
        minBest = metric(*it1,*it2);
      }
    }
//    std::cout << "row " << r << ", col " << c <<"; value v1 " << v1[r] << " v2 " << v2[c] << ", cost " << distances(r,c) 
//      << ", values " << v1[r] << ":" << v2[c] << std::endl;
    // Updates the row/col indices r and c toward the minimum between 
    // cells (r-1,c), (r,c-1) and (r-1,c-1)
    rprev = r;
    cprev = c;
    minDist = distances(r,c);
//    std::cout << " current minDist " << minDist << std::endl;
    if (rprev > 0 && distances(rprev-1,cprev) <= minDist) {
      r = rprev - 1;
      minDist = distances(rprev-1,cprev);
      ITERATOR_POS(it1,beg1,rprev-1);
      ITERATOR_POS(it2,beg2,cprev);
    }
    else if (rprev == 0 && cprev > 0) {
      c = cprev - 1;
      ITERATOR_POS(it1,beg1,rprev);
      ITERATOR_POS(it2,beg2,cprev-1);
    }
//    (rprev > 0) && std::cout << " row up " << (rprev-1) << "," << cprev << " cost " << distances(rprev-1,cprev) << ", new cost" << minDist << std::endl;
    if (cprev > 0) 
    { 
      It1 newIt1;
      It2 newIt2;
      ITERATOR_POS(newIt1,beg1,rprev);
      ITERATOR_POS(newIt2,beg2,cprev-1);
      if (distances(rprev,cprev-1) < minDist ||
          (distances(rprev,cprev-1) == minDist && 
           ((r == rprev && c == cprev) || metric(*newIt1,*newIt2) <= metric(*it1,*it2))
          )
         )
      {
        c = cprev - 1;
        minDist = distances(rprev,cprev-1);
        ITERATOR_POS(it1,beg1,rprev);
        ITERATOR_POS(it2,beg2,cprev-1);
      }
    }
    else if (cprev == 0 && rprev > 0) {
      r = rprev - 1;
      ITERATOR_POS(it1,beg1,rprev-1);
      ITERATOR_POS(it2,beg2,cprev);
    }
//    (cprev > 0) && std::cout << " col up " << rprev << "," << (cprev-1) << " cost " << distances(rprev,cprev-1) << ", new cost" << minDist << std::endl;
    if (rprev > 0 && cprev > 0) 
    {
      It1 newIt1;
      It2 newIt2;
      ITERATOR_POS(newIt1,beg1,rprev-1);
      ITERATOR_POS(newIt2,beg2,cprev-1);
      if (distances(rprev-1,cprev-1) < minDist ||
          (distances(rprev-1,cprev-1) == minDist && 
           ((r == rprev && c == cprev) || metric(*newIt1,*newIt2) <= metric(*it1,*it2))
          )
        )
      {
        r = rprev - 1;
        c = cprev - 1;
        ITERATOR_POS(it1,beg1,rprev-1);
        ITERATOR_POS(it2,beg2,cprev-1);
      }
    }
//    (rprev > 0 && cprev > 0) && std::cout << " diag " << (rprev-1) << "," << (cprev-1) << " cost " << distances(rprev-1,cprev-1) << ", new cost" << minDist << std::endl;
//    std::cout << " *** move from " << rprev << "," << cprev << " to " << r << "," << c << std::endl;
  } 
  assert(r == 0 && c == 0);
//  std::cout << "r " << r << " c " << c << " v1 " << v1[r] << " v2 " << v2[c]
//     << "; rprev " << rprev << ", cprev " << cprev << " v1 " << v1[rprev] << " v2 " << v2[cprev]
//     << "; rbest " << rbest << ", cbest " << cbest << " v1 " << v1[rbest] << " v2 " << v2[cbest]
//     << "; best cost " << distances(rbest,cbest) << std::endl;
  if (multipleAssociationOn) {
    ITERATOR_POS(it1,beg1,r);
    ITERATOR_POS(it2,beg2,c);
//    std::cout << "row " << r << ", col " << c << " END" << std::endl;
    associations.push_back(std::make_pair(it1,it2));
  }
  else {
//    std::cout << "row " << rbest << ", col " << cbest << ", cost " << distances(rbest,cbest) << std::endl;
//    associations.push_back(std::make_pair(rbest,cbest));
     ITERATOR_POS(it1,beg1,r);
     ITERATOR_POS(it2,beg2,c);
     if (r != rprev && c != cprev) {
//        std::cout << "  rbest " << rbest << ", cbest " << cbest << ", values " << v1[rbest] << ":" << v2[cbest] << "\n";
        ITERATOR_POS(it1,beg1,rbest);
        ITERATOR_POS(it2,beg2,cbest);
        associations.push_back(std::make_pair(it1,it2));
      }
      else if (metric(*it1,*it2) < minBest) {
        rbest = r;
        cbest = c;
        minBest = metric(*it1,*it2);
      }
     ITERATOR_POS(it1,beg1,rbest);
     ITERATOR_POS(it2,beg2,cbest);
     associations.push_back(std::make_pair(it1,it2));
  }
  std::reverse(associations.begin(),associations.end());
}

#endif

