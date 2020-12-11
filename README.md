### Description

Code to repreoduce bug in https://github.com/raisimTech/raisimLib/issues/25. Assume `$LOCAL_INSTALL=<raisim_ws>/build`

### Build
Copy `activation.raisim` in `rsc/`

`cd build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_PREFIX_PATH=$LOCAL_INSTALL`

### Run 
`cd build`

`valgrind ./collision`

