# HiveMind

The HiveMind is the embedded application that runs on SwarmUS HiveBoard and uses the HiveSight

## Building

```
mkdir build
cd build
cmake ../
make
```

### Dependencies

You can avoid cloning all the dependencies to speed up the first cmake call by specifying the path. 
```
cmake -DCMAKE_BUILD_TYPE=Debug -DCFREERTOS_KENEL_PATH=PATH -DCSTM32_CUBE_${FAMILY}_PATH=PATH <more_definition> ../
make
```