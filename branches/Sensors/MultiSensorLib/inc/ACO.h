//============================================================================
// Name        : ACO.h
// Author      : Tobias
// Version     :
// Copyright   :
// Description : use ACO slove TSP
//============================================================================
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
using namespace std;
using namespace mrpt;
using namespace mrpt::poses;

namespace tkhl{
	const int iAntCount=34;
	const int iCityCount=10; // total city count
	const int iItCount=50; // cycle count
	const double Q=100;
	const double alpha=1;
	const double beta=5;
	const double rou=0.5;

	static const int LAT = 0; //!Index for the Latitude in a CPoint2D
	static const int LON = 1; //!Index for the Longitude in a CPoint2D

	int besttour[iCityCount]; // best tour

	double TSPrnd(int low,double uper) //generate random number
	{
		double p=(rand()/(double)RAND_MAX)*((uper)-(low))+(low);

		return (p);
	}
	int rnd(int uper)
	{
		return (rand()%uper);
	}

	class TSPGInfo
	{
	public:
		double m_dDeltTrial[iCityCount][iCityCount];
		double m_dTrial[iCityCount][iCityCount];
		double distance[iCityCount][iCityCount];
	};


	TSPGInfo Map;

	class TSPAnt
	{
	private:
		int ChooseNextCity();
		double prob[iCityCount];
		int m_iCityCount;
		int AllowedCity[iCityCount];
	public:
		void addcity(int city);
		int tabu[iCityCount];//stored visited cities
		void Clear();
		void UpdateResult();
		double m_dLength;
		double m_dShortest;//shortest path
		void move();
		TSPAnt();
		void move2last();
	};

	void TSPAnt::move2last()
	{
		int i;
		for(i=0;i<iCityCount;i++)
			if (AllowedCity[i]==1)
			{
				addcity(i);
				break;
			}
	}

	void TSPAnt::Clear()
	{
		m_dLength=0;
		int i;
		for(i=0;i<iCityCount;i++)
		{
			prob[i]=0;
			AllowedCity[i]=1;
		}
		i=tabu[iCityCount-1];
		m_iCityCount=0;
		addcity(i);
	}

	TSPAnt::TSPAnt()
	{
		m_dLength=m_dShortest=0;
		m_iCityCount=0;
		int i;
		for(i=0;i<iCityCount;i++)
		{
			AllowedCity[i]=1;
			prob[i]=0;
		}
	}
	void TSPAnt::addcity(int city)
	{
		//add city to tabu;
		tabu[m_iCityCount]=city;
		m_iCityCount++;
		AllowedCity[city]=0;
	}
	int TSPAnt::ChooseNextCity()
	{
		//Update the probability of path selection
		//select a path from tabu[m_iCityCount-1] to next


		int i;
		int j=10000;
		double temp=0;
		int curCity=tabu[m_iCityCount-1];
		for (i=0;i<iCityCount;i++)
		{
			if((AllowedCity[i]==1))
			{
				temp+=pow((1.0/Map.distance[curCity][i]),beta)*pow((Map.m_dTrial[curCity][i]),alpha);
			}
		}
		double sel=0;
		for (i=0;i<iCityCount;i++)
		{
			if((AllowedCity[i]==1))
			{
				prob[i]=pow((1.0/Map.distance[curCity][i]),beta)*pow((Map.m_dTrial[curCity][i]),alpha)/temp;
				sel+=prob[i];
			}
			else
				prob[i]=0;
		}
		double mRate=TSPrnd(0,sel);
		double mSelect=0;

		for ( i=0;i<iCityCount;i++)
		{
			if((AllowedCity[i]==1))
				mSelect+=prob[i] ;
			if (mSelect>=mRate) {j=i;break;}
		}

		if (j==10000)
		{
			temp=-1;
			for (i=0;i<iCityCount;i++)
			{
				if((AllowedCity[i]==1))
					if (temp<pow((1.0/Map.distance[curCity][i]),beta)*pow((Map.m_dTrial[curCity][i]),alpha))
					{
						temp=pow((1.0/Map.distance[curCity][i]),beta)*pow((Map.m_dTrial[curCity][i]),alpha);
						j=i;
					}
			}
		}

		return j;

	}
	void TSPAnt::UpdateResult()
	{
		// Update the length of tour
		int i;
		for(i=0;i<iCityCount-1;i++)
			m_dLength+=Map.distance[tabu[i]][tabu[i+1]];
		m_dLength+=Map.distance[tabu[iCityCount-1]][tabu[0]];
	}
	void TSPAnt::move()
	{
		//the ant move to next town and add town ID to tabu.
		int j;
		j=ChooseNextCity();
		addcity(j);
	}
	class TSPDriver
	{
	public:

		void UpdateTrial();
		double m_dLength;// max 10^9
		void initmap();
		TSPAnt ants[iAntCount];
		void GetAnt();
		void StartSearch();
		TSPDriver(mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit);
	private:
		double getDistance( int mode, double lat1, double lon1, double lat2, double lon2 );
	};
	void TSPDriver::UpdateTrial()
	{
		int i;
		int j;

		for(i=0;i<iAntCount;i++)
		{
			for (j=0;j<iCityCount-1;j++)
			{
				Map.m_dDeltTrial[ants[i].tabu[j]][ants[i].tabu[j+1]]+=Q/ants[i].m_dLength ;
				Map.m_dDeltTrial[ants[i].tabu[j+1]][ants[i].tabu[j]]+=Q/ants[i].m_dLength;
			}
			Map.m_dDeltTrial[ants[i].tabu[iCityCount-1]][ants[i].tabu[0]]+=Q/ants[i].m_dLength;
			Map.m_dDeltTrial[ants[i].tabu[0]][ants[i].tabu[iCityCount-1]]+=Q/ants[i].m_dLength;
		}
		for (i=0;i<iCityCount;i++)
		{
			for (j=0;j<iCityCount;j++)
			{
				Map.m_dTrial[i][j]=(rou*Map.m_dTrial[i][j]+Map.m_dDeltTrial[i][j] );
				Map.m_dDeltTrial[i][j]=0;
			}

		}


	}
	void TSPDriver::initmap()
	{
		int i;
		int j;
		for(i=0;i<iCityCount;i++)
			for (j=0;j<iCityCount;j++)
			{

				Map.m_dTrial[i][j]=1;
				Map.m_dDeltTrial[i][j]=0;
			}
	}
	/* Test version
	TSPDriver::TSPDriver()
	{
		//initial map, read map information from coming vectors.
		initmap();
		m_dLength=10e9;

		ifstream in("eil51.tsp");

		struct city
		{
			int num;
			int x;
			int  y;
		}cc[iCityCount];

		for (int i=0;i<iCityCount;i++)
		{
			in>>cc[i].num>>cc[i].x>>cc[i].y;
			besttour[i]=0;
		}
		int j;
		for(int i=0;i<iCityCount;i++)
			for (j=0;j<iCityCount;j++)
			{
				{
					Map.distance[i][j]=sqrt(pow((cc[i].x-cc[j].x),2)+pow((cc[i].y-cc[j].y),2));
				}
			}


	}
	*/

	//initial map, read map information from coming vectors.
	TSPDriver::TSPDriver(mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t & toVisit)
	{

		initmap();
		m_dLength=10e9;


		struct city
		{
			int num;
			int x;
			int y;
		}cc[iCityCount];

		for (int i=0;i<iCityCount;i++)
		{
			cc[i].num = i+1;
			cc[i].x = toVisit[i].m_coords[LAT];
			cc[i].y = toVisit[i].m_coords[LON];
			besttour[i]=0;
		}
		int j;
		for(int i=0;i<iCityCount;i++)
			for (j=0;j<iCityCount;j++)
			{
				{
					//Map.distance[i][j]=sqrt(pow((cc[i].x-cc[j].x),2)+pow((cc[i].y-cc[j].y),2));
					Map.distance[i][j]=getDistance(0, cc[i].x, cc[i].y, cc[j].x, cc[j].y);
				}
			}


	}

	void TSPDriver::GetAnt()
	{
		//randomly put ant into map
		int i=0;
		int city;
		srand( (unsigned)time( NULL ) +rand());
		for (i=0;i<iAntCount;i++)
		{
			city=rnd(iCityCount);
			ants[i].addcity(city);
		}

	}


	void TSPDriver::StartSearch()
	{
		//begin to find best solution
		int max=0;//every ant tours times
		int i;
		int j;
		double temp;
		int temptour[iCityCount];
		while (max<iItCount)
		{
			for(j=0;j<iAntCount;j++)

			{
				for (i=0;i<iCityCount-1;i++)
					ants[j].move();
			}

			for(j=0;j<iAntCount;j++)
			{
				ants[j].move2last();
				ants[j].UpdateResult ();
			}

			//find out the best solution of the step and put it into temp
			int t;
			temp=ants[0].m_dLength;
			for (t=0;t<iCityCount;t++)
				temptour[t]=ants[0].tabu[t];
			for(j=0;j<iAntCount;j++)
			{
				if (temp>ants[j].m_dLength) {
					temp=ants[j].m_dLength;
					for ( t=0;t<iCityCount;t++)
						temptour[t]=ants[j].tabu[t];
				}

			}

			if(temp<m_dLength){
				m_dLength=temp;
				for ( t=0;t<iCityCount;t++)
					besttour[t]=temptour[t];
			}
			//printf("%d : %f\n",max,m_dLength);
			UpdateTrial();

			for(j=0;j<iAntCount;j++)
				ants[j].Clear();

			max++;

		}
		/*
		printf("The shortest toure is : %f\n",m_dLength);

		for ( int t=0;t<iCityCount;t++)
			printf(" %d ",besttour[t]);
		*/
	}

	double TSPDriver::getDistance(int mode, double lat1, double lon1, double lat2, double lon2 ) {
				if(mode == 1){
					return sqrt(pow((lat1-lat2), 2) + pow((lon1-lon2), 2));
				}
				else{
					const double EARTH_RADIUS_AVERAGE = 6371009;//average earth radius in meters

					double deltalat = lat2 - lat1;
					double deltalon = lon2 - lon1;

					//convert from degree to radian
					deltalat *= M_PI/180.0;
					deltalon *= M_PI/180.0;
					lat1 *= M_PI/180.0;
					lat2 *= M_PI/180.0;

					//nasty spherical trig stuff
					double a = pow( sin( deltalat / 2 ), 2 ) +
							cos(lat1) * cos(lat2) * pow( sin( deltalon / 2 ), 2 );
					double c = 2 * atan2( sqrt(a), sqrt(1-a) );

					return EARTH_RADIUS_AVERAGE * c;
				}
			}
}
