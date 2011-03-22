all:
	gcc-4.2 -fno-strict-aliasing -fno-common -dynamic -arch i386 -arch x86_64 -g -O2 -DNDEBUG -g -O3 -I/opt/local/include/ -I/Library/Frameworks/Python.framework/Versions/2.7/include/python2.7 -c Camera.cpp -o build/temp.macosx-10.6-intel-2.7/Camera.o
	c++ -arch i386 -arch x86_64 -isysroot / -g -bundle -undefined dynamic_lookup -arch i386 -arch x86_64 -isysroot / -g build/temp.macosx-10.6-intel-2.7/Camera.o -L/opt/local/lib -lopencv_imgproc -lopencv_core -lopencv_highgui -o build/lib.macosx-10.6-intel-2.7/camera.so
	python setup.py install