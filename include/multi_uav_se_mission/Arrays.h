/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#ifndef MULTI_UAV_SE_MISSION_ARRAYS_H
#define MULTI_UAV_SE_MISSION_ARRAYS_H

#include <vector>
#include <algorithm>

namespace multi_uav_se_mission{

class Arrays {

  private:

	public:
    Arrays();
    ~Arrays();

    template <class VecType>
    static std::vector<VecType> concatenate(std::vector<VecType> a, std::vector<VecType> b);

    template <class VecType>
    static void emplaceBack(std::vector<VecType> &a, std::vector<VecType> b);

    template <class VecType>
    static std::vector<VecType> subvector(std::vector<VecType> vec, int start, int size);

    template <class T>
    static void removeByValue(std::vector<T> &vec, T value);

    template <class T>
    static bool contains(std::vector<T> vec, T value);

};

////////////////////////////////////////////////////
// Template functions need to be
// implemented in the header file
////////////////////////////////////////////////////

template <class VecType>
std::vector<VecType> Arrays::concatenate(std::vector<VecType> a, std::vector<VecType> b){

  std::vector<VecType> ab;

  ab.reserve(a.size() + b.size());  // preallocate memory

  ab.insert( ab.end(), a.begin(), a.end() );
  ab.insert( ab.end(), b.begin(), b.end() );

  return ab;

}

template <class VecType>
void Arrays::emplaceBack(std::vector<VecType> &a, std::vector<VecType> b){

  a.reserve(a.size() + b.size());  // preallocate memory

  a.insert( a.end(), b.begin(), b.end() );
}

template <class VecType>
std::vector<VecType> Arrays::subvector(std::vector<VecType> vec, int start, int size){
  std::vector<VecType> subvec;

  if((start + size) > vec.size()) return subvec;

  for (std::size_t i = start; i<(start + size); i++) {
    subvec.push_back(vec[i]);
  }

  return subvec;
}

template <class T>
void Arrays::removeByValue(std::vector<T> &vec, T value){
    vec.erase(std::remove(vec.begin(), vec.end(), value), vec.end());
}

template <class T>
bool Arrays::contains(std::vector<T> vec, T value){

  if(std::find(vec.begin(), vec.end(), value) != vec.end()) {
    return true;
  } else {
    return false;
  }

}

}

#endif // MULTI_UAV_SE_MISSION_ARRAYS_H
