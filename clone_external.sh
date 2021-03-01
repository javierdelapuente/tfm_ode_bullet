#!/bin/bash

git clone https://github.com/javierdelapuente/qhull.git extern/qhull
cd extern/qhull
git checkout master
cd -
git clone https://github.com/javierdelapuente/ode.git extern/ode
cd extern/ode
git checkout master
cd -
git clone https://github.com/javierdelapuente/bullet3.git extern/bullet3
cd extern/bullet3
git checkout master
cd -
git clone https://github.com/javierdelapuente/ogre.git extern/ogre
cd extern/ogre
git checkout v1.12.8
cd -
git clone https://github.com/javierdelapuente/googletest.git extern/googletest
cd extern/googletest
git checkout release-1.10.0
cd -
git clone https://github.com/javierdelapuente/eigen extern/eigen
cd extern/eigen
git checkout master
cd -
