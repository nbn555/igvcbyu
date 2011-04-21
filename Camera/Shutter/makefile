run : 
	g++ -o bin/test src/CameraFeed.cpp -I inc -I /usr/include/opencv -I /usr/include/flycapture -L/usr/lib -lcv -lcxcore -lhighgui -lflycapture
	bin/test

seg : 
	g++ -o bin/seg src/ColorSegment.cpp -I inc -I /usr/include/opencv -I /usr/include/flycapture -L/usr/lib -lcv -lcxcore -lhighgui -lflycapture
	bin/seg

compile: 
	g++ -o bin/test src/CameraFeed.cpp -I inc -I /usr/include/opencv -I /usr/include/flycapture -L/usr/lib -lcv -lcxcore -lhighgui -lflycapture

valgrind:
	valgrind --tool=memcheck --leak-check=yes --max-stackframe=5000000 --show-reachable=yes ./test

clean : 
	@- rm -f bin/test
