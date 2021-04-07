#ifndef __IBITTYBUZZNEIGHBORSMANAGER_H_
#define __IBITTYBUZZNEIGHBORSMANAGER_H_

class IBittyBuzzNeighborsManager {
  public:
    virtual ~IBittyBuzzNeighborsManager() = default;

    virtual void updateNeighbors() = 0;
};

#endif // __IBITTYBUZZNEIGHBORSMANAGER_H_
