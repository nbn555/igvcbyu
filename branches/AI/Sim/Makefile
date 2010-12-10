all: LibYclops.so Headers ReactiveNavigationDemoMain Libs

Libs: link.txt
	cp link.txt ~/mrpt-src/apps/ReactiveNavigationDemo/CMakeFiles/ReactiveNavigationDemo.dir

ReactiveNavigationDemoMain: ReactiveNavigationDemoMain.cpp
	cp ReactiveNavigationDemoMain.cpp ~/mrpt-src/apps/ReactiveNavigationDemo/

Headers: Yclopsnav.h
	sudo cp Yclopsnav.h /usr/include

LibYclops.so: YclopsNavigationSystem.o
	 g++ -shared -Wl,-soname,LibYclops.so  YclopsNavigationSystem.o -lc -o ~/mrpt/lib/LibYclops.so

YclopsNavigationSystem.o: YclopsNavigationSystem.cpp
	g++ -fPIC -g -c YclopsNavigationSystem.cpp -o YclopsNavigationSystem.o

