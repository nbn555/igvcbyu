/**
 * @file UnitTest.h
 * @date Feb 22, 2011
 * @author tallred3
 * @brief Unit test base class
 */

#ifndef UNITTEST_H_
#define UNITTEST_H_

class UnitTest {
public:
	static bool testCompass();
private:
	UnitTest(){};
	virtual ~UnitTest(){};
};

#endif /* UNITTEST_H_ */
