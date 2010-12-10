all: LibYclops.so Headers ReactiveNavigationDemoMain

ReactiveNavigationDemoMain: ReactiveNavigationDemoMain.cpp
	cp ReactiveNavigationDemoMain.cpp ~/mrpt-src/apps/ReactiveNavigationDemo/

Headers: Yclopsnav.h
	sudo cp Yclopsnav.h /usr/include

LibYclops.so: YclopsNavigationSystem.o
	 g++ -shared -Wl,-soname,LibYclops.so  YclopsNavigationSystem.o -lc -o ~/mrpt/lib/LibYclops.so

YclopsNavigationSystem.o: YclopsNavigationSystem.cpp
	g++ -fPIC -g -c YclopsNavigationSystem.cpp -o YclopsNavigationSystem.o

