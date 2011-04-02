/**
 * @file View.h
 * @date Mar 25, 2011
 * @author tallred3
 * @brief 
 */

#ifndef VIEW_H_
#define VIEW_H_


#include <string>
#include "AbstractView.h"
#include "AbstractController.h"
#include "Observable.h"
//#include <ncurses.h>

typedef struct _win_st WINDOW;	//this was required for as to not use the ncurses header which for some reason causes a conflict with mrpt

class NView: public MVC::AbstractView {
public:
	NView(util::Observable * m, MVC::AbstractController * c = NULL);
	virtual ~NView();
	void update( util::Observable * m );
	void log( const std::string & str );
protected:
	WINDOW * log_win;
	WINDOW * data_win;
	WINDOW * ai_win;
	int row;

	WINDOW * setup_window( int starty, int startx, int height, int width );
	void updateDataWindow(WINDOW* win, double lat, double lon, double yaw );
	void updateLogWindow( WINDOW* win, const std::string& mesg);
	void updateAiWindow( WINDOW* win );

};

#endif /* VIEW_H_ */
