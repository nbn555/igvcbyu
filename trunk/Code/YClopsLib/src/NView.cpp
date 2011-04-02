/**
 * @file View.cpp
 * @date Mar 25, 2011
 * @author tallred3
 * @brief 
 */

#include "NView.h"
#include <string.h>
#include "YClopsModel.h"
#include "logging.h"
#include <ncurses.h>

using namespace std;
using namespace MVC;
using namespace util;

NView::NView(util::Observable * m, AbstractController * c ): AbstractView(m,c), row(0) {

	Log::setView(this);

	int height = 0, width = 0;

	initscr();
	cbreak();
	noecho();
	nodelay(stdscr,true);
	getmaxyx(stdscr,height, width);
	refresh();
	this->log_win = setup_window(0, 0, height, width/2);
	this->data_win = setup_window( 0, width/2, height/2, width/2);
	this->ai_win = setup_window( height/2, width/2, height/2, width/2);
	scrollok(log_win, true);

	string str;

	this->updateDataWindow(this->data_win, 0, 0, 0);
	this->updateLogWindow(this->log_win, str);
	this->updateAiWindow(this->ai_win);

}

NView::~NView() {
	endwin();
	delwin(log_win);
	delwin(data_win);
	delwin(ai_win);
}
void NView::update( util::Observable * m ) {

	YClopsModel * model = ((YClopsModel*)m);

	this->updateDataWindow(this->data_win, 0, 0, 0 );
	this->updateAiWindow(this->ai_win);
}

void NView::log( const std::string & str ) {
	this->updateLogWindow(this->log_win, str);
}

WINDOW * NView::setup_window( int starty, int startx, int height, int width ) {
	WINDOW * local_window;
	local_window = newwin(height,width, starty, startx);
	wrefresh(local_window);
	return local_window;
}

void NView::updateDataWindow(WINDOW* win, double lat, double lon, double yaw) {
	char buffer[32];
	mvwprintw(win, 0, 1, "Latitude: ");
	mvwprintw(win, 1, 1, "Longitude: ");
	mvwprintw(win, 2, 1, "Yaw: ");

	sprintf(buffer, "%f", lat);
	mvwprintw(win, 0, 11, buffer);
	sprintf(buffer, "%f", lon);
	mvwprintw(win, 1, 11, buffer);
	sprintf(buffer, "%f", yaw);
	mvwprintw(win, 2, 11, buffer);
	wrefresh(win);

}

void NView::updateLogWindow( WINDOW* win, const string & mesg ) {
	int height = 0, width = 0;
	getmaxyx(win,height,width);

	int len = mesg.length();

//	if('\n' == mesg[len-1] ) {
//		mesg[len-1] = '\0';
//		len--;
//	}

	const int lines = len>0?len/width+1:0;

	if(this->row >= height) {
		wscrl(win,lines);
		this->row -= lines;
	}

	mvwprintw(win, this->row, 1, mesg.c_str());

	wrefresh(win);
	this->row += lines;

}

void NView::updateAiWindow( WINDOW* win ) {

}
