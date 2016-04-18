## This is the project Energy Monitoring program compatible the odroidxu3.
### How to build the Energy Monitor Qt application.

#### Install packages and clone the source code.

```
# sudo apt-get install qt4-default libqwt-dev
# git clone https://github.com/hardkernel/EnergyMonitor.git
```

#### Create a project file.
```
# qmake -project
```

#### Add below CONFIG line in the project file.
```
# vi EnergyMonitor.pro
```
######...
######CONFIG += qwt
######...

#### Make!
```
# qmake
# make -j8
```
