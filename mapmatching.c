// Trakm8 Map Matching v.0.4.21x4

/*
 * TODO
 *
 * using OSM and HERE map source like parameter in  function [now it is hardcoded]
 * using constant ANGLE_TOLERANCE_NEARBY by type of data. If new data [gyro_dif] set to 20, if old data [no gyro_dif] set to 26
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include "mapmatching.h"

#ifdef TESTING_TIMES
#include <sys/time.h>
#endif	//TESTING_TIMES



static double dist2(double v_lon, double v_lat, double w_lon, double w_lat) { return pow(v_lon - w_lon,2) + pow(v_lat - w_lat,2); }
static double AVG_M_PER_S_FROM_KPH[8] = {2.77778, 5.55556,	11.11111, 16.66667, 22.22222, 26.38889, 31.94444, 36.11111};
static double AVG_M_PER_S_FROM_MPH[8] = {2.23520, 5.36448,	11.17600, 15.64640, 21.01088, 26.82240, 32.18688, 35.76320};

void pointOnSegment(double p_lon, double p_lat, double v_lon, double v_lat, double w_lon, double w_lat, double *f_lon, double *f_lat)
{

	double l2 = dist2(v_lon, v_lat, w_lon, w_lat);
	if (l2 == 0)
	{
		*f_lon = v_lon;
		*f_lat = v_lat;
		return;
	}

	double t = ((p_lon- v_lon) * (w_lon - v_lon) + (p_lat - v_lat) * (w_lat- v_lat)) / l2;

	if (t < 0)
	{
		*f_lon = v_lon;
		*f_lat = v_lat;
		return;
	}

	if (t > 1)
	{
		*f_lon = w_lon;
		*f_lat = w_lat;
		return;
	}

	*f_lon = v_lon + t * (w_lon - v_lon);
	*f_lat = v_lat + t * (w_lat - v_lat);

	return;
}


double vzd_by_degree(double lon1, double lat1, double lon2, double lat2)
{
    /*double a1 =   (lat1 - lat2)*DEGREE_LENGTH;
    double a2 = pow (a1,2);
    double b1 = (lon1 - lon2)*DEGREE_LENGTH;
    double b2 = cos( lat1 * (0.0175));
    double b3 = pow ((b1 * b2),2);
    double c = sqrt(a2+b3);
    double d = sqrt(pow(((lat1 - lat2)*DEGREE_LENGTH),2) + pow((((lon1 - lon2)*DEGREE_LENGTH)*(cos( lat1 * (0.0175)))),2));    //0.175 = PI/180 for rad calc*/

	return sqrt(pow(((lat1 - lat2)*DEGREE_LENGTH),2) + pow((((lon1 - lon2)*DEGREE_LENGTH)*(cos( lat1 * (0.0174533)))),2));    //0.175 = PI/180 for rad calc
}

double distToLine(double lon, double lat, double l_lon1, double l_lat1,double l_lon2, double l_lat2){
	double point_on_line_lon, point_on_line_lat;

	pointOnSegment(lon,  lat,  l_lon1,  l_lat1, l_lon2,  l_lat2, &point_on_line_lon, &point_on_line_lat);

	return vzd_by_degree(lon,lat,point_on_line_lon, point_on_line_lat);
}

/*
static double HaversineDistance(double lon1, double lat1, double lon2, double lat2)
{
#ifdef TESTING
	HaversineDistance_count++;
#endif	//TESTING

	const double EartRadiusKm = 6371;
    double d2r = (M_PI/ 180);
    double dlon = (lon2 - lon1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat / 2.0), 2) + cos(lat1 * d2r) * cos(lat2 * d2r) * pow(sin(dlon / 2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return c * EartRadiusKm*1000;
}
*/

double FindPath(Node *nodes, int nodesCount, int StartNode1,int StartNode2, int targetNode)
{
#ifdef TESTING_COUNTS
	FindPath_count++;
#endif	//TESTING_COUNTS

	int numberOfOpenListItems,i,j, activeFcost, neighborsNode, activeNode,neighborsActive,dist_to_S1, dist_to_S2;

#ifdef ROUTING_OPT
	double max_dist_acc, max_dist;
#endif


	dist_to_S1 = vzd_by_degree(nodes[StartNode1].x,nodes[StartNode1].y, nodes[targetNode].x,nodes[targetNode].y);
	dist_to_S2= vzd_by_degree(nodes[StartNode2].x,nodes[StartNode2].y, nodes[targetNode].x,nodes[targetNode].y);
	// Fill two nodes as start nodes, Gcost is already filled, Fcost is calculated, put them to open list
	nodes[StartNode1].openList=TRUE;
	nodes[StartNode1].Fcost=nodes[StartNode1].Gcost+dist_to_S1;
	nodes[StartNode2].openList=TRUE;
	nodes[StartNode2].Fcost=nodes[StartNode2].Gcost+dist_to_S2;
	numberOfOpenListItems = 2;
	if(StartNode1 == StartNode2)
	{
		numberOfOpenListItems = 1;
	}

#ifdef ROUTING_OPT

	dist_to_S1 = vzd_by_degree(nodes[StartNode1].x,nodes[StartNode1].y, nodes[targetNode].x,nodes[targetNode].y);
	dist_to_S2= vzd_by_degree(nodes[StartNode2].x,nodes[StartNode2].y, nodes[targetNode].x,nodes[targetNode].y);
	if(dist_to_S1 > dist_to_S2)
		max_dist = dist_to_S1;
	else
		max_dist = dist_to_S2;
	max_dist_acc = max_dist*2+200;

	//if(max_dist_to_stop < 500) max_dist_to_stop =500;
#endif

	do{
		if (numberOfOpenListItems != 0 ){

			j=-1; //no 0, wil be incremented in do loop

			// finding activeNode - the node with smalles Fcost, only for nodes on open and NOT on close list
			activeFcost = INT_MAX;
			for(i=0;i<numberOfOpenListItems;i++){
				do{ j++; }while(nodes[j].openList != TRUE || nodes[j].closeList == TRUE);
				if( nodes[j].Fcost < activeFcost){
					activeFcost = nodes[j].Fcost;
					activeNode = j;
				}
			}
			// we have the winner, Gcost is the distance
			if (activeNode == targetNode){
				//clearing open and close Lists
				for(i=0;i<nodesCount;i++){
					nodes[i].openList=FALSE;
					nodes[i].closeList=FALSE;
				}
				return nodes[targetNode].Gcost;
			}
#ifdef ROUTING_OPT
			if(nodes[activeNode].Gcost > max_dist_acc){
				for(i=0;i<nodesCount;i++){
					nodes[i].openList=FALSE;
					nodes[i].closeList=FALSE;
				}
				return -1;
			}
#endif
			// put active node to close list
			nodes[activeNode].closeList = TRUE;
			numberOfOpenListItems--;
			// adding neghbors nodes
			for(neighborsActive = 0; neighborsActive < nodes[activeNode].neighborsCount; neighborsActive++){ 	//fo through all neighbors
				neighborsNode = nodes[activeNode].neighborsID[neighborsActive];
	   			if (nodes[neighborsNode].closeList != TRUE){   //only for not on close list
					if (nodes[neighborsNode].openList != TRUE){   // if not on open lis - add it to open list and calculate Gcost and Fcost and set parent
						nodes[neighborsNode].openList = TRUE;
						numberOfOpenListItems++;
						nodes[neighborsNode].Gcost=nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
						nodes[neighborsNode].Fcost=nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x,nodes[neighborsNode].y, nodes[targetNode].x,nodes[targetNode].y);
						nodes[neighborsNode].parent=activeNode;
					}else{		//if already on open list - check if this new node has better G than previous
						if (nodes[neighborsNode].Gcost>nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive]){  // if yes callculate G, F, parent and replace
							nodes[neighborsNode].Gcost=nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
							nodes[neighborsNode].Fcost=nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x,nodes[neighborsNode].y, nodes[targetNode].x,nodes[targetNode].y);
							nodes[neighborsNode].parent=activeNode;
						}
					}
				}
			}
		}else{		// no way, buddy
			for(i=0;i<nodesCount;i++){
				nodes[i].openList=FALSE;
				nodes[i].closeList=FALSE;
			}
			return -1;
		}

	}
	while (1);//Do until path is found or deemed nonexistent

	return -1;
}

//points_drawn += draw_points_on_line(line, mm_Start, map_pts, first_length, def_length, draw_points, up_down);
int draw_points_on_line(MM_WKBLINE* line, int first_point, MM_MAP_PT_W* map_pts, double draw_at, double def_length, int points, int up_down)
{
	int  points_drawn=0,i;

	double  vzd_start, vzd_end, vzd;
#ifdef HEADING
	int heading;
	double degree_coef;
#endif	//HEADING

			vzd_end = 0;
	for(i = 0; i<line->num_points-1; i++){
		vzd_start = vzd_end;
		vzd= vzd_by_degree(line->points[i].x, line->points[i].y,line->points[i+1].x, line->points[i+1].y);
		vzd_end += vzd;
#ifdef	DEBUG
		//printf("i: %i num: %i\n",i,line->num_points);
#endif
		while(vzd_end > draw_at && points_drawn < points){

#ifdef	DEBUG
		//printf("%i %i %i %f %f %f %f \n",points_drawn,points,first_point+(up_down*(points_drawn)),line->points[i+1].x,line->points[i].x,line->points[i+1].y,line->points[i].y);
#endif
			map_pts[first_point+(up_down*(points_drawn))].longitude=line->points[i].x+(line->points[i+1].x-line->points[i].x)*((double)(draw_at-vzd_start)/vzd);
			map_pts[first_point+(up_down*(points_drawn))].latitude=line->points[i].y+(line->points[i+1].y-line->points[i].y)*((double)(draw_at-vzd_start)/vzd);
			map_pts[first_point+(up_down*(points_drawn))].geocoded = 1;
#ifdef HEADING
			degree_coef = cos(map_pts[first_point+(up_down*(points_drawn))].latitude * (0.0174533));
			heading = 90-(atan2(line->points[i+1].y - line->points[i].y, (line->points[i+1].x - line->points[i].x) * degree_coef) * (180 / M_PI));
			map_pts[first_point+(up_down*(points_drawn))].heading = heading;
			map_pts[first_point+(up_down*(points_drawn))].line_id = 0;
			map_pts[first_point+(up_down*(points_drawn))].road_dist = 0;
#endif	//HEADING
			points_drawn++;
			draw_at += def_length;
		}
		if(points_drawn == points)
			break;
	}
#ifdef	DEBUG
		//printf("ddd i: %i num: %i\n",i,line->num_points);
#endif
		for(; points_drawn< points; points_drawn++){
		map_pts[first_point+(up_down*points_drawn)].longitude=line->points[i].x;
		map_pts[first_point+(up_down*points_drawn)].latitude=line->points[i].y;
		map_pts[first_point+(up_down*(points_drawn))].geocoded = 1;
#ifdef HEADING
		map_pts[first_point+(up_down*(points_drawn))].heading = 0;
		map_pts[first_point+(up_down*(points_drawn))].line_id = 0;
		map_pts[first_point+(up_down*(points_drawn))].road_dist = 0;
#endif	//HEADING
	}
	return points_drawn;
}

int draw_points_on_lineA(MM_WKBLINE* line, int mm_Start, int mm_End, MM_MAP_PT_W* map_pts)
{
	int  points_drawn=0,i;
	int points, first_point;
	double def_lenght = 0.0, draw_at,vzd_start, vzd_end, vzd;//,vzd_1,vzd_2,lon,lat;
	int up_down;
#ifdef HEADING
	int heading;
	double degree_coef;
#endif	//HEADING
	points = mm_End - mm_Start -1;
	def_lenght = fabs(map_pts[mm_Start].dist_to_start-map_pts[mm_End].dist_to_start)/(points+1);
	if(map_pts[mm_Start].dist_to_start<map_pts[mm_End].dist_to_start){
		first_point = mm_Start;
		//last_point = mm_End;
		up_down =1;
	}else{
		//last_point = mm_Start;
		first_point = mm_End;
		up_down = -1;
	}

	draw_at = map_pts[first_point].dist_to_start+def_lenght;
	vzd_end = 0;
	for(i = 0; i<line->num_points-1; i++){
		vzd_start = vzd_end;
		vzd= vzd_by_degree(line->points[i].x, line->points[i].y,line->points[i+1].x, line->points[i+1].y);
		vzd_end += vzd;

		while(vzd_end > draw_at && points_drawn < points){
			map_pts[first_point+(up_down*(points_drawn +1))].longitude=line->points[i].x+(line->points[i+1].x-line->points[i].x)*((double)(draw_at-vzd_start)/vzd);
			map_pts[first_point+(up_down*(points_drawn +1))].latitude=line->points[i].y+(line->points[i+1].y-line->points[i].y)*((double)(draw_at-vzd_start)/vzd);
			map_pts[first_point+(up_down*(points_drawn +1))].geocoded = 1;
#ifdef HEADING
			degree_coef = cos(map_pts[first_point+(up_down*(points_drawn+1))].latitude * (0.0174533));
			heading = 90-(atan2(line->points[i+1].y - line->points[i].y, (line->points[i+1].x - line->points[i].x) * degree_coef) * (180 / M_PI));
			map_pts[first_point+(up_down*(points_drawn +1))].heading = heading;
			map_pts[first_point+(up_down*(points_drawn +1))].line_id = 0;
			map_pts[first_point+(up_down*(points_drawn +1))].road_dist = 0;
#endif	//HEADING
			points_drawn++;
			draw_at += def_lenght;
		}
		if(points_drawn == points)
			break;
	}
	for(; points_drawn< points; points_drawn++){
		map_pts[first_point+(up_down*(points_drawn +1))].longitude=line->points[i].x;
		map_pts[first_point+(up_down*(points_drawn +1))].latitude=line->points[i].y;
		map_pts[first_point+(up_down*(points_drawn +1))].geocoded = 1;
#ifdef HEADING
		map_pts[first_point+(up_down*(points_drawn+1))].heading = 0;
		map_pts[first_point+(up_down*(points_drawn+1))].line_id = 0;
		map_pts[first_point+(up_down*(points_drawn+1))].road_dist = 0;
#endif	//HEADING

	}
	return points_drawn;
}

int IsSame(MM_POINT_LL p0, MM_POINT_LL p1)
{
	if(p0.x == p1.x && p0.y == p1.y)
		return 1;
	return 0;
}

void AddPointsToRoute(MM_POINT_LL* route_line, int *route_line_number, MM_WKBLINE* line, int* only_one_route_line)
{
	MM_POINT_LL temp;
	int i;
	//int count_ret = 0;

	if(line == NULL)
			return;

	if(*route_line_number == 0){
		for(i = 0; i<line->num_points; i++){
			route_line[*route_line_number]=line->points[i];
			(*route_line_number)++;
		}
		return;
	}
	if(*only_one_route_line == 1){
		if(IsSame(route_line[0],line->points[0]) && IsSame(route_line[*route_line_number-1],line->points[line->num_points-1]))
		{
			return;
		}
		if(IsSame(route_line[0],line->points[0]) || IsSame(route_line[0],line->points[line->num_points-1]))
		{
			for(i = 0; i<((*route_line_number)/2); i++){
				temp=route_line[((*route_line_number)-1) - i];
				route_line[((*route_line_number)-1) - i]=route_line[i];
				route_line[i]=temp;
			}
		}
		*only_one_route_line = 0;
	}

	if((IsSame(route_line[*route_line_number-1],line->points[0]) && IsSame(route_line[*route_line_number-2],line->points[1])) || (IsSame(route_line[*route_line_number-1],line->points[line->num_points-1]) && IsSame(route_line[*route_line_number-2],line->points[line->num_points-2])))
	{
		return;
	}

	if(IsSame(route_line[*route_line_number-1],line->points[0]))
	{
		for(i = 1; i<line->num_points; i++){
			route_line[*route_line_number]=line->points[i];
			(*route_line_number)++;
		}
	}else if(IsSame(route_line[*route_line_number-1],line->points[line->num_points-1]))
	{
		for(i = line->num_points-2; i>=0; i--){
			route_line[*route_line_number]=line->points[i];
			(*route_line_number)++;
		}
	}else{


	while(!IsSame(route_line[*route_line_number-1],line->points[0]) && !IsSame(route_line[*route_line_number-1],line->points[line->num_points-1]) && (*route_line_number)>0)
	{
		/*if(count_ret==4)
		{
			(*route_line_number)+=2;
			break;
		}
		count_ret++;*/
		(*route_line_number)--;
	}

	if((*route_line_number)){
		AddPointsToRoute(route_line, route_line_number, line, only_one_route_line);
	}else{
#ifdef	DEBUG
		printf("Error: 2 new points to line_route route_line_number: %i \n",*route_line_number);
#endif
	}
	}

}


int FindPointsOnPath(Node *nodes, int nodesCount, int mm_Start,int mm_End, int targetNode, MM_MAP_PT_W* map_pts,MM_POINT_LL* route_line, int* route_line_number,int* only_one_route_line)//, xdb_result_t *xdb_result)
{
#ifdef TESTING_COUNTS
	FindPath_count++;
#endif	//TESTING_COUNTS

	int numberOfOpenListItems,i,j,k, activeFcost, neighborsNode, activeNode,neighborsActive,points_in_turn;

	int  points_drawn=0, StartNode1,StartNode2, EndNode1,points, draw_points,up_down,mm_s,index;//EndNode2,
	double way_length, draw_at,line_length, def_length, rest_at,line_length_best,vzd_1,vzd_2,lat,lon;//rest_dist,
	MM_WKBLINE *line;

	/*map_pts[j].longitude = tslots[j].points[c].x;
	map_pts[j].latitude = tslots[j].points[c].y;
	map_pts[j].geocoded = 1;
	map_pts[j].line_id = tslots[j].line_id[c];
	map_pts[j].start_Node = tslots[i].start_Node[c];
	map_pts[j].end_Node = tslots[i].end_Node[c];
	map_pts[j].dist_to_start=tslots[i].dist_to_start[c];
	map_pts[j].dist_start_end=tslots[i].dist_start_end[c];*/

	if(map_pts[mm_Start].line_id == map_pts[mm_End].line_id)
	{

		line = map_pts[mm_Start].line;
		AddPointsToRoute(route_line, route_line_number, line,only_one_route_line);
		return draw_points_on_lineA(line, mm_Start, mm_End, map_pts);
		//return 0;
	}


	StartNode1 = map_pts[mm_Start].start_Node;
	StartNode2 = map_pts[mm_Start].end_Node;
	EndNode1 = map_pts[mm_End].start_Node;
	//EndNode2 = map_pts[mm_End].end_Node;
	// Fill two nodes as start nodes, Gcost is already filled, Fcost is calculated, put them to open list
	nodes[StartNode1].openList=TRUE;
	nodes[StartNode1].Fcost=nodes[StartNode1].Gcost+vzd_by_degree(nodes[StartNode1].x,nodes[StartNode1].y, nodes[targetNode].x,nodes[targetNode].y);
	nodes[StartNode2].openList=TRUE;
	nodes[StartNode2].Fcost=nodes[StartNode2].Gcost+vzd_by_degree(nodes[StartNode2].x,nodes[StartNode2].y, nodes[targetNode].x,nodes[targetNode].y);
	numberOfOpenListItems = 2;
	if(StartNode1 == StartNode2)
	{
		numberOfOpenListItems = 1;
	}

	do{
		if (numberOfOpenListItems != 0){

			j=-1; //no 0, wil be incremented in do loop

			// finding activeNode - the node with smalles Fcost, only for nodes on open and NOT on close list
			activeFcost = INT_MAX;
			for(i=0;i<numberOfOpenListItems;i++){
				do{ j++; }while(nodes[j].openList != TRUE || nodes[j].closeList == TRUE);
				if( nodes[j].Fcost < activeFcost){
					activeFcost = nodes[j].Fcost;
					activeNode = j;
				}
			}
			// we have the winner, Gcost is the distance
			if (activeNode == targetNode){
				//clearing open and close Lists
				for(i=0;i<nodesCount;i++){
					nodes[i].openList=FALSE;
					nodes[i].closeList=FALSE;
				}
				/////////////////////////

				//xdb_free_result(xdb_result);

				way_length = nodes[targetNode].Gcost;
				points = mm_End - mm_Start -1;
				def_length = way_length/(points+1);
				way_length -= def_length;
				mm_s = mm_End -1;

				for(i=0;i<nodes[nodes[activeNode].parent].neighborsCount;i++){
					if(nodes[nodes[activeNode].parent].neighborsID[i]==activeNode){
						line_length = map_pts[mm_End].dist_start_end;

						if(nodes[activeNode].parent==EndNode1){ //  start node
// OK
							if(def_length < 0.1)
								draw_points = points;
							else{
								draw_points = (map_pts[mm_End].dist_to_start/def_length);
								if(draw_points > points)
									draw_points = points;
							}
							draw_at = map_pts[mm_End].dist_to_start - (draw_points * def_length);
							up_down=1;
							mm_s = mm_s - draw_points+1;
							line = map_pts[mm_End].line;//nodes[nodes[activeNode].parent].line[i];
							points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
							//rest_dist = way_length - (def_length*draw_points);
							rest_at = draw_at;
							mm_s = mm_s - 1;
						}
						else
						{





//OK
							if(def_length < 0.1)
								draw_points = points;
							else{
								draw_points = ((line_length-map_pts[mm_End].dist_to_start)/def_length) ;
								if(draw_points > points)
									draw_points = points;
							}
///TODO - why is there more points??? test data  3.3.
							//draw_at =  line_length - map_pts[mm_End].dist_to_start + def_length;
							draw_at =  map_pts[mm_End].dist_to_start + def_length;
									//nodes[nodes[activeNode].parent].neighborsDistance[i] - (def_length*(draw_points+1));
							up_down=-1;

							line = map_pts[mm_End].line;//nodes[nodes[activeNode].parent].line[i];

#ifdef	DEBUG
	//	printf("1c %i %i %i %i %i %f %f %f \n",points,mm_End ,mm_Start,mm_s,draw_points,line_length,map_pts[mm_End].dist_to_start,def_length);

#endif
							points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
							mm_s = mm_s - points_in_turn ;
							//rest_dist = way_length - (def_length*draw_points);
							rest_at = (double)line_length - (def_length*(points_in_turn-1)) - draw_at;

						}

						/*vzd_1=vzd_by_degree(map_pts[mm_End-points_drawn].longitude,map_pts[mm_End-points_drawn].latitude,map_pts[mm_End-(points_drawn+1)].longitude,map_pts[mm_End-(points_drawn+1)].latitude);
						vzd_2=vzd_by_degree(map_pts[mm_End-points_drawn].longitude,map_pts[mm_End-points_drawn].latitude,map_pts[mm_End-(points_drawn+points_in_turn)].longitude,map_pts[mm_End-(points_drawn+points_in_turn)].latitude);

						if(vzd_2 < vzd_1)
						{
							for(k=0;k<points_in_turn/2;k++)
							{
								lon = map_pts[mm_End-(points_drawn+1+k)].longitude;
								lat = map_pts[mm_End-(points_drawn+1+k)].latitude;
								map_pts[mm_End-(points_drawn+1+k)].longitude=map_pts[mm_End-(points_drawn+points_in_turn-k)].longitude;
								map_pts[mm_End-(points_drawn+1+k)].latitude=map_pts[mm_End-(points_drawn+points_in_turn-k)].latitude;
								map_pts[mm_End-(points_drawn+points_in_turn-k)].longitude=lon;
								map_pts[mm_End-(points_drawn+points_in_turn-k)].latitude=lat;
							}
						}*/
						points_drawn = points_in_turn;
						break;
					}
				}

				AddPointsToRoute(route_line, route_line_number, line, only_one_route_line);


				activeNode = nodes[activeNode].parent;

				while((StartNode2 != activeNode && StartNode1 !=activeNode) ){//&& points_drawn<points){

					line_length_best = DBL_MAX;
					for(i=0;i<nodes[nodes[activeNode].parent].neighborsCount;i++){
						if(nodes[nodes[activeNode].parent].neighborsID[i]==activeNode){
							line_length = nodes[nodes[activeNode].parent].neighborsDistance[i];
							if(line_length < line_length_best){
							   line_length_best = line_length;
							   index = i;
							}
						}
					}
					line = nodes[nodes[activeNode].parent].line[index];
					AddPointsToRoute(route_line, route_line_number, line,only_one_route_line);
					if(points_drawn<points){
						if(def_length < 0.1)
							draw_points = points-points_drawn;
						else{
							if((int)((line_length+rest_at)/def_length)>(points-points_drawn))
								draw_points = points-points_drawn;
							else
								draw_points = (line_length+rest_at)/def_length;
						}


						if(nodes[nodes[activeNode].parent].startNode[index] != 1){
							up_down=-1;
							//up_down=1;
							//mm_s = mm_s - draw_points+1;
							draw_at = def_length - rest_at;// line_length - (draw_points * def_length) + rest_at;//
							points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
							//mm_s = mm_s - 1 ;
							mm_s = mm_s - points_in_turn;
							rest_at =  line_length + rest_at - (def_length*points_in_turn);
						}else{
							//up_down=-1;
							up_down=1;
							mm_s = mm_s - draw_points+1;
							draw_at = line_length - (draw_points * def_length) + rest_at;
							points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
							mm_s = mm_s - 1 ;
							//mm_s = mm_s - points_in_turn;
							rest_at =  line_length + rest_at - (def_length*points_in_turn);
						}
						/*if(nodes[nodes[activeNode].parent].startNode[index] != 1){
													up_down=1;  /////star
						//problem
													mm_s = mm_s - draw_points+1;
													draw_at = def_length - rest_at;// line_length - (draw_points * def_length) + rest_at;//
													points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
													mm_s = mm_s - 1 ;
													rest_at =  line_length + rest_at - (def_length*points_in_turn);
												}else{
													up_down=-1;
						//ssss
													//mm_s = mm_s - draw_points;
													draw_at = line_length - (draw_points * def_length) + rest_at;
													points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
													mm_s = mm_s - points_in_turn;
													rest_at =  line_length + rest_at - (def_length*points_in_turn);
												}*/
						//rest_dist = rest_dist - (def_length*draw_points);
//#ifdef	DEBUG
						vzd_1=vzd_by_degree(map_pts[mm_End-points_drawn].longitude,map_pts[mm_End-points_drawn].latitude,map_pts[mm_End-(points_drawn+1)].longitude,map_pts[mm_End-(points_drawn+1)].latitude);
						vzd_2=vzd_by_degree(map_pts[mm_End-points_drawn].longitude,map_pts[mm_End-points_drawn].latitude,map_pts[mm_End-(points_drawn+points_in_turn)].longitude,map_pts[mm_End-(points_drawn+points_in_turn)].latitude);

						if(vzd_2 < vzd_1 && points_in_turn>1)
						{
							for(k=0;k<points_in_turn/2;k++)
							{
								lon = map_pts[mm_End-(points_drawn+1+k)].longitude;
								lat = map_pts[mm_End-(points_drawn+1+k)].latitude;
								map_pts[mm_End-(points_drawn+1+k)].longitude=map_pts[mm_End-(points_drawn+points_in_turn-k)].longitude;
								map_pts[mm_End-(points_drawn+1+k)].latitude=map_pts[mm_End-(points_drawn+points_in_turn-k)].latitude;
								map_pts[mm_End-(points_drawn+points_in_turn-k)].longitude=lon;
								map_pts[mm_End-(points_drawn+points_in_turn-k)].latitude=lat;
							}
						}
//#endif
						points_drawn += points_in_turn;
					}
					activeNode = nodes[activeNode].parent;
				}

				line = map_pts[mm_Start].line;
				AddPointsToRoute(route_line, route_line_number, line, only_one_route_line);
				if(points_drawn<points){
					line_length = map_pts[mm_Start].dist_start_end;

						if(activeNode !=StartNode1){
							draw_points = points-points_drawn;
							draw_at = map_pts[mm_Start].dist_start_end-(def_length*(draw_points-1))  - def_length + rest_at;
							up_down=1;
							mm_s = mm_s - draw_points+1;


							points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
						}
						else
						{
							draw_points = points-points_drawn;
							draw_at =  def_length - rest_at;
							up_down=-1;
							//mm_s = mm_s - draw_points ;


							points_in_turn = draw_points_on_line(line, mm_s, map_pts, draw_at, def_length, draw_points, up_down);
						}

					/*	vzd_1=vzd_by_degree(map_pts[mm_End-points_drawn].longitude,map_pts[mm_End-points_drawn].latitude,map_pts[mm_End-(points_drawn+1)].longitude,map_pts[mm_End-(points_drawn+1)].latitude);
						vzd_2=vzd_by_degree(map_pts[mm_End-points_drawn].longitude,map_pts[mm_End-points_drawn].latitude,map_pts[mm_End-(points_drawn+points_in_turn)].longitude,map_pts[mm_End-(points_drawn+points_in_turn)].latitude);

						if(vzd_2 < vzd_1)
						{
							for(k=0;k<points_in_turn/2;k++)
							{
								lon = map_pts[mm_End-(points_drawn+1+k)].longitude;
								lat = map_pts[mm_End-(points_drawn+1+k)].latitude;
								map_pts[mm_End-(points_drawn+1+k)].longitude=map_pts[mm_End-(points_drawn+points_in_turn-k)].longitude;
								map_pts[mm_End-(points_drawn+1+k)].latitude=map_pts[mm_End-(points_drawn+points_in_turn-k)].latitude;
								map_pts[mm_End-(points_drawn+points_in_turn-k)].longitude=lon;
								map_pts[mm_End-(points_drawn+points_in_turn-k)].latitude=lat;
							}
						}*/
						points_drawn += points_in_turn;

				}
		//xdb_free_result(xdb_result);
				return points_drawn;
				//////////////////////////////////


			}
			// put active node to close list
			nodes[activeNode].closeList = TRUE;
			numberOfOpenListItems--;
			// adding neghbors nodes
			for(neighborsActive = 0; neighborsActive < nodes[activeNode].neighborsCount; neighborsActive++){ 	//fo through all neighbors
				neighborsNode = nodes[activeNode].neighborsID[neighborsActive];
	   			if (nodes[neighborsNode].closeList != TRUE){   //only for not on close list
					if (nodes[neighborsNode].openList != TRUE){   // if not on open lis - add it to open list and calculate Gcost and Fcost and set parent
						nodes[neighborsNode].openList = TRUE;
						numberOfOpenListItems++;
						nodes[neighborsNode].Gcost=nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
						nodes[neighborsNode].Fcost=nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x,nodes[neighborsNode].y, nodes[targetNode].x,nodes[targetNode].y);
						nodes[neighborsNode].parent=activeNode;
                				 }else{		//if already on open list - check if this new node has better G than previous
						if (nodes[neighborsNode].Gcost>nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive]){  // if yes callculate G, F, parent and replace
							nodes[neighborsNode].Gcost=nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
							nodes[neighborsNode].Fcost=nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x,nodes[neighborsNode].y, nodes[targetNode].x,nodes[targetNode].y);
							nodes[neighborsNode].parent=activeNode;
						}
					}
				}
			}
		}else{		// no way, buddy
			for(i=0;i<nodesCount;i++){
				nodes[i].openList=FALSE;
				nodes[i].closeList=FALSE;
			}
			return -1;
		}
	}
	while (1);//Do until path is found or deemed nonexistent

	return -1;
}
/*
int same_point(POINT_LL p1, POINT_LL p2){
	if((p1.x == p2.x) && (p1.y == p2.y))
		return 1;
	return 0;
}


int is_part_ok(MM_WKBLINE *list_line, int num_list_line, int mm_Start,int mm_End,MM_MAP_PT_W* map_pts,TIMESLOT* tslots)
{

	int  points_drawn=0,i,point;
	int points, first_point;
	double def_lenght = 0.0, draw_at,vzd_start, vzd_end, vzd;//,vzd_1,vzd_2,lon,lat;
	int up_down, i_list_line,dist_sum, prvni,dist,part_dist;
#ifdef HEADING
	int heading;
	double degree_coef;
#endif	//HEADING


	//map_pts[mm_End].latitude

	if(same_point(list_line[0].points[0],list_line[1].points[0]) || same_point(list_line[0].points[0],list_line[1].points[list_line[1].num_points-1]))
	{
			prvni = 0;
	}else{
			prvni = 1;
	}

	draw_at= map_pts[mm_End].dist_to_start;
	point = mm_End;

	for(i_list_line = 0;i_list_line<num_list_line;i_list_line++ )
	{
		degree_coef = cos(list_line[i_list_line].points[i].y * (0.0174533));
		if(i_list_line)
		{
			if(same_point(list_line[i_list_line-1].points[0],list_line[i_list_line].points[0]) ||
				same_point(list_line[i_list_line-1].points[0],list_line[i_list_line].points[list_line[i_list_line+1].num_points-1]))
			{
				prvni = 0;
			}else{
				prvni = 1;
			}

			dist_sum = 0;

			if(prvni)
			{

				for(i = 0; i<list_line[i_list_line].num_points-1; i++)
				{
					part_dist = vzd_by_degree(list_line[i_list_line].points[i].x, list_line[i_list_line].points[i].y,list_line[i_list_line].points[i+1].x, list_line[i_list_line].points[i+1].y);
					dist_sum += part_dist;
					while(dist_sum>draw_at && point < mm_Start)
					{

						map_pts[point].longitude=list_line[i_list_line].points[i].x+(list_line[i_list_line].points[i+1].x-list_line[i_list_line].points[i].x)*((double)(part_dist-(dist_sum - draw_at))/part_dist);
						map_pts[point].latitude=list_line[i_list_line].points[i].y+(list_line[i_list_line].points[i+1].y-list_line[i_list_line].points[i].y)*((double)(part_dist-(dist_sum - draw_at))/part_dist);
						map_pts[point].geocoded = 1;
						heading = 90-(atan2(list_line[0].points[i+1].y - list_line[0].points[i].y, (list_line[0].points[i+1].x - list_line[0].points[i].x) * degree_coef) * (180 / M_PI));
						map_pts[point].heading = heading;


						vzd= vzd_by_degree(tslots[point]->gps_pt->longitude,tslots[point]->gps_pt->latitude,tslots[point+1]->gps_pt->longitude,tslots[point+1]->gps_pt->latitude);
						draw_at += vzd;

					}
				}
			}
		}

	}

}


int FindPointsOnPath2(Node *nodes, int nodesCount, int mm_Start,int mm_End, int targetNode, MM_MAP_PT_W* map_pts, TIMESLOT* tslots)//, xdb_result_t *xdb_result)
{

	int numberOfOpenListItems,i,j,k, activeFcost, neighborsNode, activeNode,neighborsActive,points_in_turn;

	int  points_drawn=0, StartNode1,StartNode2, EndNode1,points, draw_points,up_down,mm_s,index;//EndNode2,
	double way_length, draw_at,line_length, def_length, rest_at,line_length_best,vzd_1,vzd_2,lat,lon;//rest_dist,
	MM_WKBLINE *list_line[100];
	int num_list_line;


	StartNode1 = map_pts[mm_Start].start_Node;
	StartNode2 = map_pts[mm_Start].end_Node;
	EndNode1 = map_pts[mm_End].start_Node;
	nodes[StartNode1].openList=TRUE;
	nodes[StartNode1].Fcost=nodes[StartNode1].Gcost+vzd_by_degree(nodes[StartNode1].x,nodes[StartNode1].y, nodes[targetNode].x,nodes[targetNode].y);
	nodes[StartNode2].openList=TRUE;
	nodes[StartNode2].Fcost=nodes[StartNode2].Gcost+vzd_by_degree(nodes[StartNode2].x,nodes[StartNode2].y, nodes[targetNode].x,nodes[targetNode].y);
	numberOfOpenListItems = 2;
	if(StartNode1 == StartNode2)
	{
		numberOfOpenListItems = 1;
	}

	do{
		if (numberOfOpenListItems != 0){

			j=-1; //no 0, wil be incremented in do loop

			// finding activeNode - the node with smalles Fcost, only for nodes on open and NOT on close list
			activeFcost = INT_MAX;
			for(i=0;i<numberOfOpenListItems;i++){
				do{ j++; }while(nodes[j].openList != TRUE || nodes[j].closeList == TRUE);
				if( nodes[j].Fcost < activeFcost){
					activeFcost = nodes[j].Fcost;
					activeNode = j;
				}
			}
			// we have the winner, Gcost is the distance
			if (activeNode == targetNode){
				//clearing open and close Lists
				for(i=0;i<nodesCount;i++){
					nodes[i].openList=FALSE;
					nodes[i].closeList=FALSE;
				}
				/////////////////////////

				num_list_line=0;
				way_length = nodes[targetNode].Gcost;
				list_line[num_list_line] = map_pts[mm_End].line;
				num_list_line++;
				activeNode = nodes[activeNode].parent;
				while(StartNode2 != activeNode && StartNode1 !=activeNode){
					line_length_best = DBL_MAX;
					for(i=0;i<nodes[nodes[activeNode].parent].neighborsCount;i++){
						if(nodes[nodes[activeNode].parent].neighborsID[i]==activeNode){
							line_length = nodes[nodes[activeNode].parent].neighborsDistance[i];
							if(line_length < line_length_best){
							   line_length_best = line_length;
							   index = i;
							}
						}
					}
					list_line[num_list_line] = nodes[nodes[activeNode].parent].line[index];
					num_list_line++;
					activeNode = nodes[activeNode].parent;
				}

				list_line[num_list_line] = map_pts[mm_Start].line;//nodes[nodes[activeNode].parent].line[i];
				num_list_line++;

				is_part_ok(list_line, num_list_line,  mm_Start, mm_End, map_pts, tslots);


			}
			// put active node to close list
			nodes[activeNode].closeList = TRUE;
			numberOfOpenListItems--;
			// adding neghbors nodes
			for(neighborsActive = 0; neighborsActive < nodes[activeNode].neighborsCount; neighborsActive++){ 	//fo through all neighbors
				neighborsNode = nodes[activeNode].neighborsID[neighborsActive];
	   			if (nodes[neighborsNode].closeList != TRUE){   //only for not on close list
					if (nodes[neighborsNode].openList != TRUE){   // if not on open lis - add it to open list and calculate Gcost and Fcost and set parent
						nodes[neighborsNode].openList = TRUE;
						numberOfOpenListItems++;
						nodes[neighborsNode].Gcost=nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
						nodes[neighborsNode].Fcost=nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x,nodes[neighborsNode].y, nodes[targetNode].x,nodes[targetNode].y);
						nodes[neighborsNode].parent=activeNode;
                				 }else{		//if already on open list - check if this new node has better G than previous
						if (nodes[neighborsNode].Gcost>nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive]){  // if yes callculate G, F, parent and replace
							nodes[neighborsNode].Gcost=nodes[activeNode].Gcost + nodes[activeNode].neighborsDistance[neighborsActive];
							nodes[neighborsNode].Fcost=nodes[neighborsNode].Gcost + vzd_by_degree(nodes[neighborsNode].x,nodes[neighborsNode].y, nodes[targetNode].x,nodes[targetNode].y);
							nodes[neighborsNode].parent=activeNode;
						}
					}
				}
			}
		}else{		// no way, buddy
			for(i=0;i<nodesCount;i++){
				nodes[i].openList=FALSE;
				nodes[i].closeList=FALSE;
			}
			return -1;
		}
	}
	while (1);//Do until path is found or deemed nonexistent

	return -1;
}

*/

#ifdef HEADING
static int snap_to_segment(MM_WKBLINE *line, double gps_lon, double gps_lat, double *lon_snap, double *lat_snap, double *shortest_distance, double* dist_to_start,	double degree_coef,	int *heading)
#else
static int snap_to_segment(MM_WKBLINE *line, double gps_lon, double gps_lat, double *lon_snap, double *lat_snap, double *shortest_distance, double* dist_to_start,	double degree_coef)
#endif	//HEADING
{


	double distance = 1000.0; ///< current distance between GPS location and candidate solution
    //double distance_to_mm = 0.0; /// distance from start to best candidate
    double d_mm = 0.0; /// distance from start to current part

    double p_lon, p_lat, /*lon_snap_degraded,*/ v_lon, v_lat, w_lon, w_lat, res_lon, res_lat;
    int i = 0;

    *shortest_distance = distance; ///< candidate distance

    if(line->num_points < 2) {
        return -1;
    }

    // For each pair of points
    for(i = 0; i<line->num_points-1; i++)
    {
    	p_lon = gps_lon*degree_coef;
    	p_lat = gps_lat;
    	v_lon = line->points[i].x*degree_coef;
    	v_lat = line->points[i].y;
    	w_lon = line->points[i+1].x*degree_coef;
    	w_lat = line->points[i+1].y;
       pointOnSegment(p_lon, p_lat, v_lon, v_lat, w_lon, w_lat, &res_lon, &res_lat);
	res_lon = res_lon / degree_coef;

        distance = vzd_by_degree(gps_lon,gps_lat, res_lon, res_lat);
        if(distance < *shortest_distance) {
            // Found a better solution, use this one
            *lon_snap = res_lon;
            *lat_snap = res_lat;
            *shortest_distance = distance;
#ifdef HEADING
            *heading = 90- (atan2(w_lat - v_lat, (w_lon - v_lon)) * (180.0 / M_PI));// heading from box has 0 no north and clockwise
            //printf ("%f,%f %f,%f %i\n",v_lat,v_lon/degree_coef, *lat_snap, lon_snap_degraded/degree_coef,*heading );
#endif	//HEADING
            *dist_to_start/*distance_to_mm*/ = d_mm + vzd_by_degree(line->points[i].x,line->points[i].y, res_lon, res_lat);
        }
        d_mm += vzd_by_degree(line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);
   }

   // *shortest_distance = vzd_by_degree(*lon_snap,*lat_snap,gps_lon,gps_lat);

    //*dist_to_start = distance_to_mm;

    return 1;
}

double get_emission_probability(double distance, double emis_param)
{
	//return 1 * (EMIS_COEF * exp (-0.5 * pow(distance / emis_param, 2)));
	/*double k=1;
	if(distance>emis_param)
		k=emis_param/distance;
	return k;*/
	double ret;
	ret = (1 / sqrt(2 * M_PI * emis_param)) * exp (-0.5 * pow(distance / emis_param, 2));
	if(ret == 0.0)
		ret = 1E-299;
	return ret;
	//return (1 / sqrt(2 * M_PI * EMIS_PARAM)) * exp (-0.5 * pow(distance / EMIS_PARAM, 2));
}

double get_transition_probability(double dist_excess/*, double trans_param*/)
//double get_transition_probability(double dist,double route_dist, double trans_param)
{
#ifdef TESTING_COUNTS
	get_transition_probability_count++;
#endif	//TESTING_COUNTS

/*	double k=1;
	if(route_dist<1)route_dist = 1;
	if(dist<1)dist = 1;
	if(dist<route_dist){
		k=(dist+((route_dist-dist)/4))/route_dist;
	}else{
		k=route_dist/dist;
	}
	return k;*/
	//return 1 * (TRANS_COEF * exp (-(dist_excess) / trans_param ));
	return (1 / TRANS_PARAM) * exp (-(dist_excess) / TRANS_PARAM );
	//return (1 / trans_param) * exp (-(dist_excess) / trans_param );
}


/**********************************************************************************************************************
* Fill up start and end time slots. Some GPS points can be abandoned, so than count of time slots will be less than
* gps_count depending on number of abandoned points.
* @in gps_pts			Array of GPS points.
* @in gps_count			Count of GPS points.
* @out tslot			Array of time slots.
* @return				Total distance between all GPS points (degrees).
**********************************************************************************************************************/
void init_time_slots(MM_GPS_PT_W* gps_pts, int gps_count, TIMESLOT* tslots)
{
	int i;

	for (i = 0; i < gps_count-1; i++)	//compute all distances
	{
		tslots[i].gps_pt = &gps_pts[i];


		if ((gps_pts[i].latitude == gps_pts[i+1].latitude) && (gps_pts[i].longitude == gps_pts[i+1].longitude))
		{
			tslots[i].dist_to_next = 0;
		}
		else
		{
			tslots[i].dist_to_next = vzd_by_degree(gps_pts[i].longitude, gps_pts[i].latitude, gps_pts[i+1].longitude, gps_pts[i+1].latitude);
		}
	}

	tslots[gps_count-1].gps_pt = &gps_pts[gps_count-1];		//last time slot

	return;
}


/**********************************************************************************************************************
* Create set of possible map matched points for all GPS points (one gps point is covered by one timeslot), and prepare
* data for routing.
* @in/out  tslot	Array of time slots, for which will be candidates searched.
* @in count			Count of time slots.
* @return			Return code.
**********************************************************************************************************************/
MM_ERRORS create_candidates_set(TIMESLOT* tslots, IMM_MAP_SOURCE map_source, int count, Node* nodes, int *nodes_sum, xdb_result_t *xdb_result)///HERE///
{
#ifdef TESTING_COUNTS
	create_candidates_set_count++;
#endif	//TESTING_COUNTS

#ifdef HEADING
	int heading;
#endif	//HEADING
	int i,j;
	double mm_long, mm_lat, distance, dist_to_start, degree_coef;
	double start_lon, start_lat,end_lon,end_lat;
	int active_nodes, nodes_count;
	double min_lon, max_lon, min_lat, max_lat, max_dis_lon;

	int db_index,way_id, way_length, way_length_rev, worst_dist, worst_id;
	MM_WKBLINE *line;
	int StartNode, EndNode;

	nodes_count = 0;

	while(xdb_get_next_row(xdb_result))
	{
		db_index = 0;


				if (xdb_get_field_blob(xdb_result, db_index++, (void**)&line) == 0)	//get one line from result
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_int(xdb_result, db_index++, &way_id) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_int(xdb_result, db_index++, &way_length) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_double(xdb_result, db_index++, &start_lon) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_double(xdb_result, db_index++, &start_lat) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_double(xdb_result, db_index++, &end_lon) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_double(xdb_result, db_index++, &end_lat) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_int(xdb_result, db_index++, &way_length_rev) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_double(xdb_result, db_index++, &max_lon) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_double(xdb_result, db_index++, &max_lat) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_double(xdb_result, db_index++, &min_lon) == 0)
					return MM_ERR_UNKNOWN;
				if (xdb_get_field_double(xdb_result, db_index++, &min_lat) == 0)
					return MM_ERR_UNKNOWN;

		if(nodes_count == 0){
		    degree_coef = cos(start_lat * (0.0174533));
		}

		// fill nodes from lines data for routing- runs only once for all GPS points and all routing calculations
		active_nodes = nodes_count;
		for (i=0; i < active_nodes;i++)
		{
			if (nodes[i].x == start_lon && nodes[i].y == start_lat)
			{
				for (j=0; j < nodes_count;j++)
				{
					if (nodes[j].x == end_lon && nodes[j].y == end_lat)
					{
						if(way_length >= 0)
						{
								nodes[i].neighborsID[nodes[i].neighborsCount] = j;
								nodes[i].neighborsDistance[nodes[i].neighborsCount] = way_length;
								nodes[i].line[nodes[i].neighborsCount] = line;
								nodes[i].startNode[nodes[i].neighborsCount] = 1;
								nodes[i].neighborsCount++;
						}
						if(way_length_rev >= 0)
						{
						nodes[j].neighborsID[nodes[j].neighborsCount] = i;
						nodes[j].neighborsDistance[nodes[j].neighborsCount] = way_length_rev;
						nodes[j].line[nodes[j].neighborsCount] = line;
						nodes[j].startNode[nodes[j].neighborsCount] = 0;
						nodes[j].neighborsCount++;
						}
						StartNode = i;
						EndNode = j;
						break;
					}
				}
				if (j== nodes_count)
				{
					if(way_length >= 0)
					{
					nodes[i].neighborsID[nodes[i].neighborsCount] = nodes_count;
					nodes[i].neighborsDistance[nodes[i].neighborsCount] = way_length;
					nodes[i].line[nodes[i].neighborsCount] = line;
					nodes[i].startNode[nodes[i].neighborsCount] = 1;
					nodes[i].neighborsCount++;
					}
					nodes[nodes_count].x = end_lon;
					nodes[nodes_count].y = end_lat;
					if(way_length_rev >= 0)
					{
					nodes[nodes_count].neighborsCount = 1;
					nodes[nodes_count].neighborsID[nodes[nodes_count].neighborsCount-1] = i;
					nodes[nodes_count].neighborsDistance[nodes[nodes_count].neighborsCount-1] = way_length_rev;
					nodes[nodes_count].line[nodes[nodes_count].neighborsCount-1] = line;
					nodes[nodes_count].startNode[nodes[nodes_count].neighborsCount-1] = 0;
					}else{
						nodes[nodes_count].neighborsCount = 0;
					}
					StartNode = i;
					EndNode = nodes_count;

					nodes_count++;
				}
				break;
			}
		}
		if (i== active_nodes)
		{
			active_nodes = nodes_count;
			for (j=0; j < active_nodes;j++)
			{
				if (nodes[j].x == end_lon && nodes[j].y == end_lat)
				{
					nodes[nodes_count].x = start_lon;
					nodes[nodes_count].y = start_lat;
					if(way_length >= 0)
					{
					//nodes[nodes_count].x = start_lon;
					//nodes[nodes_count].y = start_lat;
					nodes[nodes_count].neighborsCount = 1;
					nodes[nodes_count].neighborsID[nodes[nodes_count].neighborsCount-1] = j;
					nodes[nodes_count].neighborsDistance[nodes[nodes_count].neighborsCount-1] = way_length;
					nodes[nodes_count].line[nodes[nodes_count].neighborsCount-1] = line;
					nodes[nodes_count].startNode[nodes[nodes_count].neighborsCount-1] = 1;


					}else{
						nodes[nodes_count].neighborsCount = 0;
					}
					if(way_length_rev >= 0)
					{
					nodes[j].neighborsID[nodes[j].neighborsCount] = nodes_count;
					nodes[j].neighborsDistance[nodes[j].neighborsCount] = way_length_rev;
					nodes[j].line[nodes[j].neighborsCount] = line;
					nodes[j].startNode[nodes[j].neighborsCount] = 0;
					nodes[j].neighborsCount++;
					}
					StartNode = nodes_count;
					EndNode = j;
					nodes_count++;
					break;
				}
			}
		}
		if(i==active_nodes && j==active_nodes)
		{
			nodes[nodes_count].x = start_lon;
			nodes[nodes_count].y = start_lat;
			if(way_length >= 0)
			{
			nodes[nodes_count].neighborsCount = 1;
			nodes[nodes_count].neighborsID[nodes[nodes_count].neighborsCount-1] = nodes_count+1;
			nodes[nodes_count].neighborsDistance[nodes[nodes_count].neighborsCount-1] = way_length;
			nodes[nodes_count].line[nodes[nodes_count].neighborsCount-1] = line;
			nodes[nodes_count].startNode[nodes[nodes_count].neighborsCount-1] = 1;
			}else{
				nodes[nodes_count].neighborsCount = 0;
			}
			StartNode = nodes_count;
			nodes_count++;
			nodes[nodes_count].x = end_lon;
			nodes[nodes_count].y = end_lat;
			if(way_length_rev >= 0)
			{
			nodes[nodes_count].neighborsCount = 1;
			nodes[nodes_count].neighborsID[nodes[nodes_count].neighborsCount-1] = nodes_count-1;
			nodes[nodes_count].neighborsDistance[nodes[nodes_count].neighborsCount-1] = way_length_rev;
			nodes[nodes_count].line[nodes[nodes_count].neighborsCount-1] = line;
			nodes[nodes_count].startNode[nodes[nodes_count].neighborsCount-1] = 0;
			}else{
				nodes[nodes_count].neighborsCount = 0;
			}
			EndNode = nodes_count;
			nodes_count++;
		}


		// fill all time slots
		for (i = 0; i < count; i++)
		{
			// find the closest point on this line for all GPS points
			// TODO add condition not to do this part for not closed points - we have boundary box for each line
			// TODO handle error return from snap function

			if(i==0){
				max_dis_lon = GPS_DISCREPANCY*(cos( tslots[i].gps_pt->latitude * (0.0174533)));
			}

			if ((tslots[i].gps_pt->latitude - GPS_DISCREPANCY < max_lat && tslots[i].gps_pt->latitude > min_lat - GPS_DISCREPANCY) &&
			(tslots[i].gps_pt->longitude - max_dis_lon < max_lon && tslots[i].gps_pt->longitude > min_lon - max_dis_lon))
			{
#ifdef HEADING
			snap_to_segment(line, tslots[i].gps_pt->longitude, tslots[i].gps_pt->latitude, &mm_long, &mm_lat, &distance, &dist_to_start, degree_coef, &heading);
#else
			snap_to_segment(line, tslots[i].gps_pt->longitude, tslots[i].gps_pt->latitude, &mm_long, &mm_lat, &distance, &dist_to_start, degree_coef);
#endif	//HEADING
			}
			else
			{
				distance = MAX_DIST_FROM_GPS;
			}

			if(distance < MAX_DIST_FROM_GPS) // is point in range?
			{
				if (tslots[i].count < MAX_CANDIDATES)  // ? already found max candidates
				{
					tslots[i].dist_start_end[tslots[i].count] = (way_length>=0)?way_length:way_length_rev;
					tslots[i].points[tslots[i].count].x = mm_long;
					tslots[i].points[tslots[i].count].y = mm_lat;
					tslots[i].distances[tslots[i].count] = distance;
					tslots[i].dist_to_start[tslots[i].count] = dist_to_start;
					tslots[i].start_Node[tslots[i].count] = StartNode;
					tslots[i].end_Node[tslots[i].count] = EndNode;
					tslots[i].line_id[tslots[i].count] = way_id;
					tslots[i].line[tslots[i].count] = line;
#ifdef HEADING
					tslots[i].heading[tslots[i].count] = heading;
#endif	//HEADING
					if (way_length >= 0 && way_length_rev>=0)
						tslots[i].one_way[tslots[i].count] = 0;
					else if (way_length < 0)
						tslots[i].one_way[tslots[i].count] = -1;
					else
						tslots[i].one_way[tslots[i].count] = 1;



					tslots[i].count++;
				}
				else
				{
					worst_dist = 0;

					for (j = 0; j < tslots[i].count; j++) //max candidates found, ? is one worse than the new one
					{
						if (tslots[i].distances[j] > worst_dist)
						{
							worst_dist = tslots[i].distances[j];
							worst_id = j;
						}
					}

					if(worst_dist > distance){
					tslots[i].dist_start_end[worst_id] = (way_length>=0)?way_length:way_length_rev;
					tslots[i].points[worst_id].x = mm_long;
					tslots[i].points[worst_id].y = mm_lat;
					tslots[i].distances[worst_id] = distance;
					tslots[i].dist_to_start[worst_id] = dist_to_start;
					tslots[i].start_Node[worst_id] = StartNode;
					tslots[i].end_Node[worst_id] = EndNode;
					tslots[i].line_id[worst_id] = way_id;
					tslots[i].line[worst_id] = line;
#ifdef HEADING
					tslots[i].heading[worst_id] = heading;
#endif	//HEADING
					if (way_length >= 0 && way_length_rev>=0)
						tslots[i].one_way[worst_id] = 0;
					else if (way_length < 0)
						tslots[i].one_way[worst_id] = -1;
					else
						tslots[i].one_way[worst_id] = 1;
					}
				}
			}
		}
#ifdef TESTING_TIMES2
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&snap_time, &tsBefore, &tsAfter);
#endif	//TESTING_TIMES2
	}

	*nodes_sum = nodes_count;
	return MM_OK;
}

void mm_close(MM_WS* ws)
{
	xdb_close(&ws->xdb);
	free(ws);
}

MM_ERRORS mm_init(MM_INIT *init_data, MM_WS** workspace)
{
	xdb_t*	xdb;

	if( NULL == (*workspace = (MM_WS*)malloc(sizeof(MM_WS))) )	// memory is freed in mm_close method
	{
		return MM_NO_MEMORY;
	}

	xdb = &((*workspace)->xdb);

	//Init DB.
	if(xdb_init2(xdb, init_data->host, init_data->user, init_data->passwd, init_data->db, init_data->port) == 1)  {
		return MM_OK;
	} else {
		return MM_NO_DB_RESPONSE;
	}


	//return MM_OK;
}


void map_match_part(TIMESLOT* tslots, MM_MAP_PT_W* map_pts, Node* nodes, int gpscount, int nodes_count, int start,MM_POINT_LL* route_line, int* route_line_number,int* only_one_route_line)//, xdb_result_t *xdb_result)	//ORIGINAL
{
	int i,l,r,c,i_break,j;//, start_line_Node = -1, end_line_Node = -2;
	long double tmp_score, max_r_score, score[MAX_CANDIDATES];
	double dist_excess, prob_r_l, route_dist;

	//double ofset_x,ofset_y;

	//double emis_param, trans_param;
	//int k;
	//double dist_from_point=DBL_MAX;
#ifdef	DEBUG
	int iii,debug_count;
	BOOL startDebug = FALSE;
	double search_lat = 50.86795986;
#endif	//DEBUG
	//emis_param = 20; // emission parameter - its average value of difference between GPS point and real position
	//trans_param = REV_LN2; // from theory

	if(gpscount == 1) // only one GPS point, find the closest point on lines
	{
		//for(i=0;i<tslots[start].count;i++){
			//if(dist_from_point > tslots[start].distances[i]){
			//map_pts[start].longitude = tslots[start].points[i].x;
			//map_pts[start].latitude = tslots[start].points[i].y;
			map_pts[start].geocoded = 0;
			//dist_from_point = tslots[start].distances[i];
			//}
		//}
		return;
	}

i_break = start;
for (i = start; i < gpscount-1+start; i++)		// i = timeslot from, i+1 = timeslot to
{

	c = -1; //init / no parent

	for (l = 0; l < tslots[i+1].count; l++)	// l = actual candidate in timeslot[i+1]
	{
		max_r_score = 0;

		score[l] = get_emission_probability(tslots[i+1].distances[l],EMIS_PARAM); //count emisson probability
#ifdef HEADING
		/*double xxx = fmod(abs(tslots[i+1].heading[l] - tslots[i+1].gps_pt->heading),90.0);
		xxx = 1-(xxx/100.0);
		xxx = pow(xxx,3);*/
	//double xxx = get_transition_probability(fmod(abs(tslots[i+1].heading[l] - tslots[i+1].gps_pt->heading),90));
//	score[l] *= get_transition_probability(fmod(abs(tslots[i+1].heading[l] - tslots[i+1].gps_pt->heading),90));
#endif	//HEADING


		for (r = 0; r < tslots[i].count; r++)	// r = candidate in timeslot[i]
		{
/*
#ifdef	DEBUG
	if(i==0 && l == 13 && r==0)
	{
		printf("ILR\n");
	}
#endif	//DEBUG
*/
			if (i == i_break/*start*/ && l==0)
			{
				tslots[i].scores[r] = get_emission_probability(tslots[i].distances[r], EMIS_PARAM); //count emisson probability fot tslot [0], only for the first run
#ifdef HEADING
				//double xxx = get_transition_probability(fmod(abs(tslots[i].heading[r] - tslots[i].gps_pt->heading),90));

			//	tslots[i].scores[r] *= get_transition_probability(fmod(abs(tslots[i].heading[r] - tslots[i].gps_pt->heading),90));

#endif	//HEADING

			}

			// calculate road distance from r to l candidate
			if(tslots[i].line_id[r] == tslots[i+1].line_id[l]) //both on same line
			{
				if(tslots[i].one_way[r]==0 || abs(tslots[i].dist_to_start[r]-tslots[i+1].dist_to_start[l])<MINMOVE){
					route_dist= abs(tslots[i].dist_to_start[r]-tslots[i+1].dist_to_start[l]);
				}

				else if(tslots[i].one_way[r]==1)
				{
					if(tslots[i].dist_to_start[r]<=	tslots[i+1].dist_to_start[l])
						route_dist= abs(tslots[i].dist_to_start[r]-tslots[i+1].dist_to_start[l]);
					else
						route_dist=MAX_D;
				}else{
					if(tslots[i].dist_to_start[r]>=tslots[i+1].dist_to_start[l])
						route_dist= abs(tslots[i].dist_to_start[r]-tslots[i+1].dist_to_start[l]);
					else
						route_dist=MAX_D;
				}
			/*	if(tslots[i].dist_to_start[r]<=tslots[i+1].dist_to_start[l]){
					//tslots[i+1].last_Node[l] = tslots[i+1].end_Node[l];
					end_line_Node = tslots[i].end_Node[r];
					start_line_Node = tslots[i+1].start_Node[l];
				}else{
					//tslots[i+1].last_Node[l] = tslots[i+1].start_Node[l];
					end_line_Node = tslots[i].start_Node[r];
					start_line_Node = tslots[i+1].end_Node[l];
				}*/
			}else {
				if(tslots[i].start_Node[r] == tslots[i+1].start_Node[l] ) // one common node / start-start
				{
					if(tslots[i].one_way[r]!=1 && tslots[i+1].one_way[l] != -1)
						route_dist= tslots[i].dist_to_start[r] + tslots[i+1].dist_to_start[l];
					else
						route_dist=MAX_D;
					//end_line_Node = tslots[i].start_Node[r];
					//start_line_Node = tslots[i+1].start_Node[l];
				}else {
					if(tslots[i].start_Node[r] == tslots[i+1].end_Node[l]) // one common node / start-end
					{
						if(tslots[i].one_way[r]!=1 && tslots[i+1].one_way[l] != 1)
							route_dist= tslots[i].dist_to_start[r] + tslots[i+1].dist_start_end[l] - tslots[i+1].dist_to_start[l];
						else
							route_dist=MAX_D;
						//end_line_Node = tslots[i].start_Node[r];
						//start_line_Node = end_line_Node;//tslots[i+i].end_Node[l];
					}else {
						if(tslots[i].end_Node[r] == tslots[i+1].start_Node[l])  // one common node / end-start
						{
							if(tslots[i].one_way[r]!=-1 && tslots[i+1].one_way[l] != -1)
								route_dist= tslots[i].dist_start_end[r] -tslots[i].dist_to_start[r] + tslots[i+1].dist_to_start[l];
							else
								route_dist=MAX_D;
							//end_line_Node = tslots[i].end_Node[r];
							//start_line_Node = tslots[i+1].start_Node[l];
						}else {
							if(tslots[i].end_Node[r] == tslots[i+1].end_Node[l])  // one common node / end-end
							{
								if(tslots[i].one_way[r]!=-1 && tslots[i+1].one_way[l] != 1)
									route_dist= tslots[i].dist_start_end[r] - tslots[i].dist_to_start[r] + tslots[i+1].dist_start_end[l] - tslots[i+1].dist_to_start[l];
								else
									route_dist=MAX_D;
								//end_line_Node = tslots[i].end_Node[r];
								//start_line_Node = end_line_Node;//tslots[i+i].end_Node[l];
							}else {   // have to use routing

									nodes[tslots[i].start_Node[r]].Gcost=tslots[i].dist_to_start[r];  //fill distance from candidate to the first start node
									nodes[tslots[i].end_Node[r]].Gcost=tslots[i].dist_start_end[r] - tslots[i].dist_to_start[r];  //fill distance from candidate to the second start node
									if(tslots[i+1].one_way[l]!=1){
									nodes[tslots[i+1].end_Node[l]].neighborsID[nodes[tslots[i+1].end_Node[l]].neighborsCount]=nodes_count;  //add new neighbors to node which is on line where target
									nodes[tslots[i+1].end_Node[l]].neighborsDistance[nodes[tslots[i+1].end_Node[l]].neighborsCount]=tslots[i+1].dist_start_end[l] - tslots[i+1].dist_to_start[l];
									nodes[tslots[i+1].end_Node[l]].neighborsCount++;
									}
									if(tslots[i+1].one_way[l]!=-1){
									nodes[tslots[i+1].start_Node[l]].neighborsID[nodes[tslots[i+1].start_Node[l]].neighborsCount]=nodes_count; //add new neighbors to node which is on line where target
									nodes[tslots[i+1].start_Node[l]].neighborsDistance[nodes[tslots[i+1].start_Node[l]].neighborsCount]=tslots[i+1].dist_to_start[l];
									nodes[tslots[i+1].start_Node[l]].neighborsCount++;
									}
									nodes[nodes_count].x=tslots[i+1].points[l].x;
									nodes[nodes_count].y=tslots[i+1].points[l].y;
									route_dist = FindPath(nodes, nodes_count+1,
											(tslots[i].one_way[r]==1)?tslots[i].end_Node[r]:tslots[i].start_Node[r],
											(tslots[i].one_way[r]==-1)?tslots[i].start_Node[r]:tslots[i].end_Node[r],nodes_count);
									/*if(tslots[i+1].end_Node[l]==end_line_Node)
										end_line_Node=tslots[i+1].start_Node[l];
									else
										end_line_Node=tslots[i+1].end_Node[l];*/

									//start_line_Node = -1;
									//end_line_Node = -2;

									if(tslots[i+1].one_way[l]!=1){
									nodes[tslots[i+1].end_Node[l]].neighborsCount--;  // delete adde neighbors for next routing
									}
									if(tslots[i+1].one_way[l]!=-1){
									nodes[tslots[i+1].start_Node[l]].neighborsCount--;
									}

								}
							}
						}
					}
				}
			if (route_dist < 0)
			{
				prob_r_l = 0;//1E-100;	//no wey
			}else
			{
				dist_excess = abs(tslots[i].dist_to_next - route_dist);
				prob_r_l = get_transition_probability(dist_excess/*, trans_param*/);
/*
				//double prob_ofset = get_emission_probability(abs(tslots[i].distances[r]-(tslots[i+1].distances[l])),5);
				//prob_r_l *= prob_ofset;
				int aaa = DEGREE_LENGTH*fabs((tslots[i].gps_pt->latitude - tslots[i].points[r].y) - (tslots[i+1].gps_pt->latitude - tslots[i+1].points[l].y));
				ofset_y = get_emission_probability(aaa,3);
				//DEGREE_LENGTH)*(cos( lat1 * (0.0174533))
				prob_r_l *= ofset_y;
				int bbb = (cos(tslots[i].gps_pt->latitude) * (0.0174533))*DEGREE_LENGTH*fabs((tslots[i].gps_pt->longitude - tslots[i].points[r].x) - (tslots[i+1].gps_pt->longitude - tslots[i+1].points[l].x));
				ofset_x = get_emission_probability(bbb,3);
				//DEGREE_LENGTH)*(cos( lat1 * (0.0174533))
				prob_r_l *= ofset_x;
*/
			/*	if(end_line_Node == tslots[i].last_Node[r]){
					prob_r_l*=1E-10;
				}*/
				//prob_r_l = get_transition_probability(tslots[i].dist_to_next, route_dist, trans_param);
				//if (prob_r_l==0) prob_r_l = 1E-100;
			}

			tmp_score = prob_r_l * tslots[i].scores[r]; // calculate score	!11

			//if(tslots[i].scores[r]>0 && tmp_score == 0 && prob_r_l > 0)
			//	tmp_score = 1E-300;

			if (max_r_score < tmp_score)  // find the best score
			{
				max_r_score = tmp_score;
				c = r;
				//tslots[i+1].last_Node[l] = start_line_Node;
			}
		}
		tslots[i+1].scores[l] = score[l] * max_r_score;  // store the best score and parent
		//if( max_r_score>0 && tslots[i+1].scores[l] == 0)
			//tslots[i+1].scores[l] = 1E-300;
		tslots[i+1].parents[l] = c;
		//if (tslots[i+1].scores[l] == 0)
		//	c = -1;

	}
	if(c == -1 || i == gpscount-2+start) //no parent found or last run of cycle for i
	{
#ifdef	DEBUG
		//printf("gps_count %i\n",gpscount);
#endif
		if (c == -1 && i != gpscount-2+start){
						i--;
		}

		tmp_score = 0;

		for (j = 0; j < tslots[i+1].count; j++)	// find the best score in the last timeslot
		{
			if (tmp_score < tslots[i+1].scores[j])
			{
				tmp_score = tslots[i+1].scores[j];
				c = j;
			}
		}

		if (c == -1)
		{
			for (j = i+1; j >= /*start+*/i_break; j--)
			{
				map_pts[j].longitude = tslots[j].gps_pt->longitude;
				map_pts[j].latitude = tslots[j].gps_pt->latitude;
				//map_pts[j].speed = tslots[j].gps_pt->speed;
				map_pts[j].geocoded = 0;
			}
		}
		else
		{
#ifdef	DEBUG
//		printf("i,ibreak %i  %i\n",i,i_break);
#endif
			for (j = i+1; j >= /*start+*/i_break; j--)		// go back and find the parents
			{
				map_pts[j].longitude = tslots[j].points[c].x;
				map_pts[j].latitude = tslots[j].points[c].y;
				map_pts[j].geocoded = 1;
				map_pts[j].line_id = tslots[j].line_id[c];
				map_pts[j].start_Node = tslots[j].start_Node[c];
				map_pts[j].end_Node = tslots[j].end_Node[c];
				map_pts[j].dist_to_start=tslots[j].dist_to_start[c];
				map_pts[j].dist_start_end=tslots[j].dist_start_end[c];
				map_pts[j].road_dist=tslots[j].distances[c];
				map_pts[j].line=tslots[j].line[c];
				map_pts[j].one_way=tslots[j].one_way[c];
				//map_pts[j].id=tslots[j].line_id[c];
				//map_pts[j].speed = tslots[j].gps_pt->speed;
#ifdef HEADING
				map_pts[j].heading = tslots[j].heading[c];

#endif	//HEADING

					c = tslots[j].parents[c];


			}
		}
		i_break=i+2;
		i++;
	}

}



	map_pts[tslots[gpscount-1].gps_pt->index].longitude = map_pts[gpscount-1].longitude;
	map_pts[tslots[gpscount-1].gps_pt->index].latitude = map_pts[gpscount-1].latitude;
	map_pts[tslots[gpscount-1].gps_pt->index].line_id = map_pts[gpscount-1].line_id;
	//map_pts[tslots[gpscount-1].gps_pt->index].id = map_pts[gpscount-1].line_id;
	map_pts[tslots[gpscount-1].gps_pt->index].dist_to_start = map_pts[gpscount-1].dist_to_start;
	map_pts[tslots[gpscount-1].gps_pt->index].dist_start_end = map_pts[gpscount-1].dist_start_end;
	map_pts[tslots[gpscount-1].gps_pt->index].road_dist = map_pts[gpscount-1].road_dist;
	map_pts[tslots[gpscount-1].gps_pt->index].line = map_pts[gpscount-1].line;
	map_pts[tslots[gpscount-1].gps_pt->index].start_Node = map_pts[gpscount-1].start_Node;
	map_pts[tslots[gpscount-1].gps_pt->index].end_Node = map_pts[gpscount-1].end_Node;
	map_pts[tslots[gpscount-1].gps_pt->index].geocoded = map_pts[gpscount-1].geocoded;
#ifdef HEADING
	map_pts[tslots[gpscount-1].gps_pt->index].heading = map_pts[gpscount-1].heading;

#endif	//HEADING
	//map_pts[tslots[gpscount-1].gps_pt->index].speed = map_pts[gpscount-1].speed;


	*route_line_number = 0;
	*only_one_route_line = 1;
	if(map_pts[tslots[gpscount-1].gps_pt->index].geocoded)
		AddPointsToRoute(route_line, route_line_number, map_pts[gpscount-1].line,only_one_route_line);
	for (i = gpscount-1; i >0; i--)
	{
#ifdef	DEBUG
		//printf("i,lineid %i  %i\n",i,map_pts[i-1].line_id);
#endif
		map_pts[tslots[i-1].gps_pt->index].longitude = map_pts[i-1].longitude;
		map_pts[tslots[i-1].gps_pt->index].latitude = map_pts[i-1].latitude;
		map_pts[tslots[i-1].gps_pt->index].line_id = map_pts[i-1].line_id;
		//map_pts[tslots[i-1].gps_pt->index].id = map_pts[i-1].line_id;
		map_pts[tslots[i-1].gps_pt->index].dist_to_start = map_pts[i-1].dist_to_start;
		map_pts[tslots[i-1].gps_pt->index].dist_start_end = map_pts[i-1].dist_start_end;
		map_pts[tslots[i-1].gps_pt->index].road_dist = map_pts[i-1].road_dist;
		map_pts[tslots[i-1].gps_pt->index].line = map_pts[i-1].line;
		map_pts[tslots[i-1].gps_pt->index].start_Node = map_pts[i-1].start_Node;
		map_pts[tslots[i-1].gps_pt->index].end_Node = map_pts[i-1].end_Node;
		map_pts[tslots[i-1].gps_pt->index].geocoded = map_pts[i-1].geocoded;
#ifdef HEADING
		map_pts[tslots[i-1].gps_pt->index].heading = map_pts[i-1].heading;

#endif	//HEADING

		//map_pts[tslots[i-1].gps_pt->index].speed = map_pts[i-1].speed;
			if(((tslots[i].gps_pt->index)-(tslots[i-1].gps_pt->index)) > 1){ // yes if there are points between index i and i+1

			//for(j=tslots[i-1].gps_pt->index+1; j<(tslots[i].gps_pt->index);j++){
				//map_pts[j].longitude = map_pts[i].longitude;
				//map_pts[j].latitude =map_pts[i].latitude;

			if(map_pts[i-1].geocoded == 0 || map_pts[i].geocoded == 0 || map_pts[tslots[i-1].gps_pt->index].geocoded == 0 || map_pts[tslots[i].gps_pt->index].geocoded == 0){
				for(j=1;j< (tslots[i].gps_pt->index-tslots[i-1].gps_pt->index);j++)
				{
					map_pts[tslots[i-1].gps_pt->index+j].geocoded = 0;
					/*map_pts[tslots[i-1].gps_pt->index+j].longitude=map_pts[i-1].longitude;
					map_pts[tslots[i-1].gps_pt->index+j].latitude=map_pts[i-1].latitude;*/
				}

			}	else{


				nodes[map_pts[tslots[i-1].gps_pt->index].start_Node].Gcost=map_pts[tslots[i-1].gps_pt->index].dist_to_start;   //tslots[i].start_Node[r]].Gcost=tslots[i].dist_to_start[r];  //fill distance from candidate to the first start node
				nodes[map_pts[tslots[i-1].gps_pt->index].end_Node].Gcost=MAX(0,map_pts[tslots[i-1].gps_pt->index].dist_start_end - map_pts[tslots[i-1].gps_pt->index].dist_to_start);  //fill distance from candidate to the second start node
				//nodes[map_pts[i-1].start_Node].Gcost=map_pts[i-1].dist_to_start;   //tslots[i].start_Node[r]].Gcost=tslots[i].dist_to_start[r];  //fill distance from candidate to the first start node
				//nodes[map_pts[i-1].end_Node].Gcost=MAX(0,map_pts[i-1].dist_start_end - map_pts[i-1].dist_to_start);  //fill distance from candidate to the second start node

				if(map_pts[i].one_way!=1){
				nodes[map_pts[tslots[i].gps_pt->index].end_Node].neighborsID[nodes[map_pts[tslots[i].gps_pt->index].end_Node].neighborsCount]=nodes_count;  //add new neighbors to node which is on line where target
				nodes[map_pts[tslots[i].gps_pt->index].end_Node].neighborsDistance[nodes[map_pts[tslots[i].gps_pt->index].end_Node].neighborsCount]=MAX(0,map_pts[tslots[i].gps_pt->index].dist_start_end - map_pts[tslots[i].gps_pt->index].dist_to_start);
				nodes[map_pts[tslots[i].gps_pt->index].end_Node].neighborsCount++;
				}
				//if(map_pts[i].one_way!=1){
				//nodes[map_pts[i].end_Node].neighborsID[nodes[map_pts[i].end_Node].neighborsCount]=nodes_count;  //add new neighbors to node which is on line where target
				//nodes[map_pts[i].end_Node].neighborsDistance[nodes[map_pts[i].end_Node].neighborsCount]=MAX(0,map_pts[i].dist_start_end - map_pts[i].dist_to_start);
				//nodes[map_pts[i].end_Node].neighborsCount++;
				//}

				if(map_pts[i].one_way!=-1){
				nodes[map_pts[tslots[i].gps_pt->index].start_Node].neighborsID[nodes[map_pts[tslots[i].gps_pt->index].start_Node].neighborsCount]=nodes_count; //add new neighbors to node which is on line where target
				nodes[map_pts[tslots[i].gps_pt->index].start_Node].neighborsDistance[nodes[map_pts[tslots[i].gps_pt->index].start_Node].neighborsCount]=map_pts[tslots[i].gps_pt->index].dist_to_start;
				nodes[map_pts[tslots[i].gps_pt->index].start_Node].neighborsCount++;
				}
				//if(map_pts[i].one_way!=-1){
				//nodes[map_pts[i].start_Node].neighborsID[nodes[map_pts[i].start_Node].neighborsCount]=nodes_count; //add new neighbors to node which is on line where target
				//nodes[map_pts[i].start_Node].neighborsDistance[nodes[map_pts[i].start_Node].neighborsCount]=map_pts[i].dist_to_start;
				//nodes[map_pts[i].start_Node].neighborsCount++;
				//}



#ifdef	DEBUG
//		if(map_pts[i-1].line_id==3337212)
//				printf("%i  %i  %i  %i\n",nodes_count+1,tslots[i-1].gps_pt->index,	tslots[i].gps_pt->index, nodes_count);
#endif
				//nodes[nodes_count].x=map_pts[i-1].longitude; // tslots[i+1].points[l].x;
				//nodes[nodes_count].y=map_pts[i-1].latitude;//  tslots[i+1].points[l].y;

				nodes[nodes_count].x=map_pts[tslots[i].gps_pt->index].longitude; // tslots[i+1].points[l].x;
				nodes[nodes_count].y=map_pts[tslots[i].gps_pt->index].latitude;//  tslots[i+1].points[l].y;


				//if(i==38)
				route_dist = FindPointsOnPath(nodes, nodes_count+1,tslots[i-1].gps_pt->index,	tslots[i].gps_pt->index, nodes_count, map_pts, route_line, route_line_number,only_one_route_line);//,xdb_result);
//				route_dist = FindPointsOnPath(nodes, nodes_count+1,tslots[i].gps_pt->index,	tslots[i-1].gps_pt->index, nodes_count, map_pts, route_line, route_line_number,only_one_route_line);//,xdb_result);
				//route_dist = FindPointsOnPath2(nodes, nodes_count+1,tslots[i-1].gps_pt->index,	tslots[i].gps_pt->index, nodes_count, map_pts, tslots);//,xdb_result);

				if(map_pts[i].one_way!=1){
				nodes[map_pts[i].end_Node].neighborsCount--;  // delete adde neighbors for next routing
				}
				if(map_pts[i].one_way!=-1){
				nodes[map_pts[i].start_Node].neighborsCount--;
				}


			}
		}else{
			if(map_pts[i-1].geocoded == 1 && map_pts[i].geocoded == 1 && (tslots[i].gps_pt->index - tslots[i-1].gps_pt->index)>0){
			nodes[map_pts[i-1].start_Node].Gcost=map_pts[i-1].dist_to_start;   //tslots[i].start_Node[r]].Gcost=tslots[i].dist_to_start[r];  //fill distance from candidate to the first start node
			nodes[map_pts[i-1].end_Node].Gcost=MAX(0,map_pts[i-1].dist_start_end - map_pts[i-1].dist_to_start);  //fill distance from candidate to the second start node

			if(map_pts[i].one_way!=1){
			nodes[map_pts[i].end_Node].neighborsID[nodes[map_pts[i].end_Node].neighborsCount]=nodes_count;  //add new neighbors to node which is on line where target
			nodes[map_pts[i].end_Node].neighborsDistance[nodes[map_pts[i].end_Node].neighborsCount]=MAX(0,map_pts[i].dist_start_end - map_pts[i].dist_to_start);
			nodes[map_pts[i].end_Node].neighborsCount++;
			}

			if(map_pts[i].one_way!=-1){
			nodes[map_pts[i].start_Node].neighborsID[nodes[map_pts[i].start_Node].neighborsCount]=nodes_count; //add new neighbors to node which is on line where target
			nodes[map_pts[i].start_Node].neighborsDistance[nodes[map_pts[i].start_Node].neighborsCount]=map_pts[i].dist_to_start;
			nodes[map_pts[i].start_Node].neighborsCount++;
			}

#ifdef	DEBUG
//		if(map_pts[i-1].line_id==3337212)
//				printf("%i  %i  %i  %i\n",nodes_count+1,tslots[i-1].gps_pt->index,	tslots[i].gps_pt->index, nodes_count);
#endif
			nodes[nodes_count].x=map_pts[i-1].longitude; // tslots[i+1].points[l].x;
			nodes[nodes_count].y=map_pts[i-1].latitude;//  tslots[i+1].points[l].y;

			//if(i==38)
			route_dist = FindPointsOnPath(nodes, nodes_count+1,tslots[i-1].gps_pt->index,	tslots[i].gps_pt->index, nodes_count, map_pts, route_line, route_line_number,only_one_route_line);//,xdb_result);
		//	route_dist = FindPointsOnPath(nodes, nodes_count+1,tslots[i].gps_pt->index,	tslots[i-1].gps_pt->index, nodes_count, map_pts, route_line, route_line_number,only_one_route_line);//,xdb_result);
			//route_dist = FindPointsOnPath2(nodes, nodes_count+1,tslots[i-1].gps_pt->index,	tslots[i].gps_pt->index, nodes_count, map_pts, tslots);//,xdb_result);

			if(map_pts[i].one_way!=1){
			nodes[map_pts[i].end_Node].neighborsCount--;  // delete adde neighbors for next routing
			}
			if(map_pts[i].one_way!=-1){
			nodes[map_pts[i].start_Node].neighborsCount--;
			}
#ifdef	DEBUG

			//	printf("pozor, doplnena routa bez vnitrniho bodu\n");
#endif
			}
		}
		//if(i==38)
			//xdb_free_result(xdb_result);
	}


}


MM_ERRORS mm_map_match_recursive(MM_WS * ws, IMM_MAP_SOURCE	map_source, MM_REQUEST_W* mm_request, MM_RESPONSE_W* mm_response, MM_POINT_LL* route_line, int* route_line_number,int* only_one_route_line)
{
	int i, nodes_count, gpscount;//, start;
	MM_ERRORS ret_code;
	Node* nodes;
	TIMESLOT* tslots;
	MM_GPS_PT_W* gps_pts;
	MM_MAP_PT_W* map_pts;

	char query[4096];//, query_part[512];

	//char query[12000], query_part[512];
	xdb_t*	xdb;
	xdb_result_t *xdb_result = NULL;
	int rows_count;
	double max_dis_lon;
	double min_long 	= DBL_MAX;
	double min_lat 		= DBL_MAX;
	double max_long 	= -DBL_MAX;
	double max_lat 		= -DBL_MAX;



	const char *query_ptr = query;


	gps_pts = mm_request->gps_pts;
	gpscount = mm_request->gps_pts_size;
	map_pts = mm_response->map_pts;


	gpscount *= 2; // only because /2 each do cycle
	do{
		gpscount /= 2;

		 min_long 	= DBL_MAX;
		 min_lat 		= DBL_MAX;
		 max_long 	= -DBL_MAX;
		 max_lat 		= -DBL_MAX;

	/*if (gpscount > MAX_GPS_COUNT)
	{
		MM_REQUEST_W 	split_mm_request;
		MM_RESPONSE_W split_mm_response;
		int split_count;
		split_count = gpscount/2;

		split_mm_request.gps_pts = gps_pts;
		split_mm_request.gps_pts_size = split_count;
		split_mm_response.map_pts = map_pts;
		mm_map_match_recursive(ws, &split_mm_request, &split_mm_response);

		split_mm_request.gps_pts = gps_pts + split_count;
		split_mm_response.map_pts = map_pts + split_count;
		split_mm_request.gps_pts_size = gpscount - split_count;
		mm_map_match_recursive(ws, &split_mm_request, &split_mm_response);

		return MM_OK;
	}*/

	ret_code = MM_OK;					// initial set-up



	//FIND BOUNDARY VALUES
		for (i = 0; i < gpscount; i++)		// count boundary of points
		{

			if(i==0){
				max_dis_lon = GPS_DISCREPANCY*(cos( gps_pts[i].latitude * (0.0174533)));
			}
			if (gps_pts[i].latitude > max_lat)
				max_lat = gps_pts[i].latitude;
			if (gps_pts[i].latitude < min_lat)
				min_lat = gps_pts[i].latitude;
			if (gps_pts[i].longitude > max_long)
				max_long = gps_pts[i].longitude;
			if (gps_pts[i].longitude < min_long)
				min_long = gps_pts[i].longitude;
		}

		///HERE///
		if (map_source == IMM_OSM){
			sprintf(query, "SELECT AsWKB(%s),%s,%s,start_lon, start_lat,end_lon,end_lat,distance_reversed,max_lon,max_lat,min_lon,min_lat "
			"FROM %s WHERE MBRIntersects(PolygonFromText(\'POLYGON((%lf %lf,%lf %lf,%lf %lf,%lf %lf,%lf %lf))\'), %s) LIMIT %i",
				NEW_LINE_ATTRIBUTE,
				OSM_ID_ATTRIBUTE,
				WAY_LENGTH_ATTRIBUTE,
				OSM_TABLE,
				min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
				max_long + max_dis_lon, max_lat + GPS_DISCREPANCY,	// top right
				max_long + max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom right
				min_long - max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom left
				min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
				NEW_LINE_ATTRIBUTE,
				MAX_DB_LINES);
		}
		else if (map_source == IMM_HERE)
		{
			sprintf(query, "SELECT AsWKB(%s),%s,%s,start_lon, start_lat,end_lon,end_lat,distance_reversed,max_lon,max_lat,min_lon,min_lat "
			"FROM %s WHERE MBRIntersects(PolygonFromText(\'POLYGON((%lf %lf,%lf %lf,%lf %lf,%lf %lf,%lf %lf))\'), %s) LIMIT %i",
				NEW_LINE_ATTRIBUTE,
				HERE_ID_ATTRIBUTE,
				WAY_LENGTH_ATTRIBUTE,
				HERE_TABLE,
				min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
				max_long + max_dis_lon, max_lat + GPS_DISCREPANCY,	// top right
				max_long + max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom right
				min_long - max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom left
				min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
				NEW_LINE_ATTRIBUTE,
				MAX_DB_LINES);
		}
		else
		{
			//TODO
		}

	xdb = &(ws->xdb);
#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&boundary_time, &tsBefore, &tsAfter);
#endif	//TESTING_TIMES1

#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif	//TESTING_TIMES1
	xdb_result = NULL;
	xdb_result = xdb_query(xdb, query_ptr);


#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsAfter);
	TimeSpecFinal(&db_time, &tsBefore, &tsAfter);
#endif	//TESTING_TIMES1

	if (xdb_result == NULL)
	{
		return MM_NO_DB_RESPONSE;
	}

#ifdef	DEBUG_SELECT
	//printf("Count of rows in result: %i   %s; \n",xdb_result->row_count, query);
	printf("Count of rows in result: %i   ; \n",xdb_result->row_count);
#endif	//DEBUG_SELECT

	rows_count = xdb_result->row_count;

	if(rows_count == 0)
	{
		xdb_free_result(xdb_result);	//ADDED
		xdb_result = NULL;
		return MM_NO_LINES;
	}
	if((rows_count >= MAX_DB_LINES) && gpscount > 1)
	{
		xdb_free_result(xdb_result);
		xdb_result = NULL;
	}

	}while((rows_count >= MAX_DB_LINES) && gpscount > 1);

	if (xdb_result == NULL) //only for sure
	{
		return MM_NO_DB_RESPONSE;
	}

	mm_request->gps_pts_size=gpscount;


#ifdef TESTING_TIMES1
	clock_gettime(CLOCK_REALTIME, &tsBefore);
#endif	//TESTING_TIMES1
	if(NULL == (nodes = (Node*)calloc((rows_count*2)+1, sizeof(Node))))	//accocate nodes for max each line has 2 points + one as target
	{
		//printf("Can\'t allocate memory for time slots.");
		if(xdb_result)
			xdb_free_result(xdb_result);
		return MM_NO_MEMORY;
	}

	if( NULL == (tslots = (TIMESLOT*)calloc(gpscount, sizeof(TIMESLOT))))	//memory for time slots
	{
		//printf("Can\'t allocate memory for time slots.");
		if(nodes)
			free(nodes);
		if(xdb_result)
			xdb_free_result(xdb_result);
		return MM_NO_MEMORY;
	}

	for (i = 0; i < gpscount; i++)		// no neighbors yet
	{
		tslots[i].count = 0;
	}

	// set *gps_pt and compute dist_to_next for all tslots
	init_time_slots(gps_pts, gpscount, tslots);



	// fill nodes and tslots
	ret_code = create_candidates_set(tslots, map_source, gpscount, nodes, &nodes_count, xdb_result);


	if (ret_code != MM_OK) // error
	{
	/*	for (i = 0; i < gpscount; i++)		// copy GPS points to result
		{
			map_pts[i].longitude = gps_pts[i].longitude;
			map_pts[i].latitude = gps_pts[i].latitude;
		}*/
		return ret_code;
	}


	map_match_part(tslots, map_pts,nodes, gpscount, nodes_count, 0, route_line, route_line_number, only_one_route_line);//, xdb_result);


	/*for (i = 0; i < gpscount; i++)
	{
		if(tslots[i].count == 0)
		{
			map_pts[i].longitude = gps_pts[i].longitude;
			map_pts[i].latitude = gps_pts[i].latitude;
		}
		else
		{
				start = i;
			for (i=start+1; i < gpscount; i++)
				{
					if(tslots[i].count == 0)
					{
						map_match_part(tslots, map_pts,nodes, i-start, nodes_count, start);
						map_pts[i].longitude = gps_pts[i].longitude;
						map_pts[i].latitude = gps_pts[i].latitude;
						break;
					}
				}
			if(i==gpscount)
				map_match_part(tslots, map_pts,nodes, i-start, nodes_count, start);

		}
	}*/

	if(xdb_result)
		xdb_free_result(xdb_result);
	if(tslots)
		free(tslots);
	if(nodes)
		free(nodes);

	return ret_code;
}






MM_ERRORS mm_map_match_old(MM_WS * ws, MM_REQUEST* mm_request, MM_RESPONSE* mm_response,MM_POINT_LL* route_line,int* route_line_number)
{

	MM_ERRORS result;
	MM_REQUEST_W mm_request_recursive;
	MM_RESPONSE_W mm_response_recursive;
	int i, gpscount, pts_skipped = 0,k,start_at, hed_last, hed_dif=0;
	MM_GPS_PT_W* gps_pts;
	MM_MAP_PT_W* mm_pts;
	double vzd_gps =0;//, ofset_lat, ofset_lon;
	int head_dif, seg_quality=0,segments_count = 0, bad_segments=0;
	//int mm_quality=0;
	//POINT_LL route_line[100000];
	//int route_line_number;
	int only_one_route_line;
#ifdef DEBUG
	int aaa;
	FILE*	fw = NULL;
	int z;
	char vypis_gps[20000],tempchar[2048];
	sprintf(vypis_gps,"[");

	fw = fopen("/home/adminuser/workspace/mmnew/Debug/src/original_filtered.csv", "w");
	FILE* fw_mm = NULL;
	char vypis_mm[20000],tempchar_mm[2048];
	sprintf(vypis_mm,"[");
	fw_mm = fopen("/home/adminuser/workspace/mmnew/Debug/src/mm_filtered.csv", "w");
	FILE* fr_mm = NULL;
	char vypis_r[50000];
	sprintf(vypis_r,"[");

	char path[1024];
	sprintf(path, "/home/adminuser/workspace/mmnew/Debug/src/mm_route%i.csv", mm_route_file_counter);
	fr_mm = fopen(path, "w");
	mm_route_file_counter++;
#endif	//DEBUG


	gpscount = mm_request->gps_pts_size;

	gps_pts = (MM_GPS_PT_W*)calloc(1+mm_request->gps_pts_size, sizeof(MM_GPS_PT_W));
	mm_pts = (MM_MAP_PT_W*)calloc(1+mm_request->gps_pts_size, sizeof(MM_MAP_PT_W));

	mm_request_recursive.gps_pts = gps_pts;
	mm_response_recursive.map_pts = mm_pts;

	mm_response->map_pts[0].latitude = mm_request->gps_pts[0].latitude;
	mm_response->map_pts[0].longitude = mm_request->gps_pts[0].longitude;
	gps_pts[0].latitude = mm_request->gps_pts[0].latitude;
	gps_pts[0].longitude = mm_request->gps_pts[0].longitude;
	gps_pts[0].heading = mm_request->gps_pts[0].heading;
	gps_pts[0].speed = mm_request->gps_pts[0].speed;
	gps_pts[0].index = 0;
	gpscount = 1;
	start_at = 0;
	hed_last = mm_request->gps_pts[0].heading;

#ifdef DEBUG_T8
	printf("Map matching starting for %i points\n",mm_request->gps_pts_size);
#endif //DEBUG_T8
	#ifdef DEBUG
		if (mm_request->gps_pts_size == 1)
		{
			mm_request_recursive.gps_pts_size = 1;
			//result = mm_map_match_recursive(ws, &mm_request_recursive, &mm_response_recursive);
		}
	#endif	//DEBUG

		if (mm_request->gps_pts_size == 1)
		{
			mm_response->map_pts[0].latitude = mm_request->gps_pts[0].latitude;
			mm_response->map_pts[0].longitude = mm_request->gps_pts[0].longitude;
		}

	for (i = 1; i < mm_request->gps_pts_size;i++)	//start from second point
	{
		vzd_gps += vzd_by_degree(mm_request->gps_pts[i-1].longitude,mm_request->gps_pts[i-1].latitude,mm_request->gps_pts[i].longitude,mm_request->gps_pts[i].latitude);
		if(mm_request->gps_pts[i].speed>10){
			hed_dif = abs(hed_last - mm_request->gps_pts[i].heading);
			if(hed_dif >180)
				hed_dif -= 360;
		}

		if (i == (mm_request->gps_pts_size)-1 || /*(fabs(gps_pts[gpscount-1].latitude - mm_request->gps_pts[i].latitude) > MAX_GPS_DIFF)*/  vzd_gps > MAX_GPS_VZD || abs(hed_dif) > MAX_HED_DIF
			/*(fabs(gps_pts[gpscount-1].longitude - mm_request->gps_pts[i].longitude) > MAX_GPS_DIFF)*/ || pts_skipped > MAX_GPS_PTS_SKIP)
		{
			if((fabs(mm_request->gps_pts[i-1].latitude - mm_request->gps_pts[i].latitude) > MAX_GPS_DIFFER) ||
					(fabs(mm_request->gps_pts[i-1].longitude - mm_request->gps_pts[i].longitude) > MAX_GPS_DIFFER) ||
					gpscount > MAX_GPS_COUNT || i == (mm_request->gps_pts_size)-1)
				{

				if(i == (mm_request->gps_pts_size)-1 ){
					gps_pts[gpscount].latitude = mm_request->gps_pts[i].latitude;
					gps_pts[gpscount].longitude = mm_request->gps_pts[i].longitude;
					gps_pts[gpscount].heading = mm_request->gps_pts[i].heading;
					gps_pts[gpscount].speed = mm_request->gps_pts[i].speed;
					gps_pts[gpscount].index = i-start_at;
					gpscount++;
					i++; //add the last point
				}else{
					if(gps_pts[gpscount-1].index != (i-1)-start_at){
					gps_pts[gpscount].latitude = mm_request->gps_pts[i-1].latitude;
					gps_pts[gpscount].longitude = mm_request->gps_pts[i-1].longitude;
					gps_pts[gpscount].heading = mm_request->gps_pts[i-1].heading;
					gps_pts[gpscount].speed = mm_request->gps_pts[i-1].speed;
					gps_pts[gpscount].index = (i-1)-start_at;
					gpscount++;
					}
				}

				mm_request_recursive.gps_pts_size = gpscount;
#ifdef DEBUG
		//printf("mm1  %i  %i  %i  %i  %i \n",i,gpscount,mm_request_recursive.gps_pts_size,gps_pts[mm_request_recursive.gps_pts_size-1].index,start_at);
#endif	//DEBUG

				result = mm_map_match_recursive(ws, IMM_OSM, &mm_request_recursive, &mm_response_recursive, route_line,route_line_number,&only_one_route_line);	///HERE///
#ifdef DEBUG
		//printf("mm2  %i  %i  %i  %i  %i \n",i,gpscount,mm_request_recursive.gps_pts_size,gps_pts[mm_request_recursive.gps_pts_size-1].index,start_at);
#endif	//DEBUG
/*
#ifdef HEADING

		 		for (k=0;k<i-start_at;k++)
				{
						if(fmod(abs(mm_response_recursive.map_pts[k].heading -  mm_request->gps_pts[start_at+k].heading),90) > 25){
							mm_quality += 1;
						}
				}

#endif	//HEADING
*/
/*
				ofset_lat=0;
				ofset_lon=0;
				for (k=0;k<mm_request_recursive.gps_pts_size;k++)
				{
					ofset_lat += gps_pts[k].latitude - mm_response_recursive.map_pts[gps_pts[k].index].latitude;
					ofset_lon += gps_pts[k].longitude - mm_response_recursive.map_pts[gps_pts[k].index].longitude;
				}


				ofset_lat/=k;
				ofset_lon/=k;

				for (k=0;k<mm_request_recursive.gps_pts_size;k++)
				{
					gps_pts[k].latitude -= ofset_lat;
					gps_pts[k].longitude -= ofset_lon;
				}
				result = mm_map_match_recursive(ws, &mm_request_recursive, &mm_response_recursive);
*/
				if(gpscount>mm_request_recursive.gps_pts_size){
					i =  gps_pts[mm_request_recursive.gps_pts_size-1].index+start_at+1;
				}



#ifdef DEBUG

	for (z=0; z < mm_request_recursive.gps_pts_size;z++)
		{
		sprintf(tempchar,"[%f,%f],", gps_pts[z].latitude, gps_pts[z].longitude);
		strcat(vypis_gps,tempchar);

		if (i % 100)
		{
			fputs(vypis_gps,fw);
			sprintf(vypis_gps, "");
		}
	}
	for (z=0; z < mm_request_recursive.gps_pts_size;z++)
	{
	sprintf(tempchar_mm,"[%f,%f],", mm_response_recursive.map_pts[gps_pts[z].index].latitude, mm_response_recursive.map_pts[gps_pts[z].index].longitude);
	strcat(vypis_mm,tempchar_mm);

	if (i % 100)
	{
		fputs(vypis_mm,fw_mm);
		sprintf(vypis_mm, "");
	}
	}

	for (z=0; z < *route_line_number;z++)
		{

		sprintf(tempchar,"[%f,%f],", route_line[z].y, route_line[z].x);
		//sprintf(tempchar,"%f,%f\n", route_line[z].y, route_line[z].x);
		strcat(vypis_r,tempchar);

		}
#endif	//DEBUG
				if(result != MM_OK)
				{
					for (k=0;k<i-start_at;k++)
					{
							mm_response->map_pts[start_at+k].latitude = mm_request->gps_pts[start_at+k].latitude;
							mm_response->map_pts[start_at+k].longitude = mm_request->gps_pts[start_at+k].longitude;
							//mm_quality += 1;
							seg_quality += 1;
							//mm_response->map_pts[start_at+k].speed = mm_request->gps_pts[start_at+k].speed;
					}
				}else{
				for (k=0;k<i-start_at;k++)
				{
					if(mm_response_recursive.map_pts[k].geocoded){
						mm_response->map_pts[start_at+k].latitude = mm_response_recursive.map_pts[k].latitude;
						mm_response->map_pts[start_at+k].longitude = mm_response_recursive.map_pts[k].longitude;
						//mm_response->map_pts[start_at+k].id = mm_response_recursive.map_pts[k].line_id;
						head_dif = abs(mm_request->gps_pts[start_at+k].heading - mm_response_recursive.map_pts[k].heading);
						head_dif = fmod(head_dif,180);
						if(head_dif>90)
							head_dif = 180 - head_dif;
						if(head_dif > 30){
							//mm_quality += 1;
							seg_quality += 1;
						}

						//mm_response->map_pts[start_at+k].speed = mm_response_recursive.map_pts[k].speed;
					}else{
						mm_response->map_pts[start_at+k].latitude = mm_request->gps_pts[start_at+k].latitude;
						mm_response->map_pts[start_at+k].longitude = mm_request->gps_pts[start_at+k].longitude;
						//mm_quality += 1;
						seg_quality += 1;
						//mm_response->map_pts[start_at+k].speed = mm_request->gps_pts[start_at+k].speed;
					}
				}
				}
			segments_count++;
			if((double)seg_quality/(i-start_at)>0.25){
				bad_segments++;
/*				for (k=0;k<i-start_at;k++)
				{
						mm_response->map_pts[start_at+k].latitude = mm_request->gps_pts[start_at+k].latitude;
						mm_response->map_pts[start_at+k].longitude = mm_request->gps_pts[start_at+k].longitude;

				}
	*/		}
#ifdef DEBUG
		printf("segment quality %f%% %i z %i \n",(1.0-((double)seg_quality/(i-start_at)))*100,seg_quality,i-start_at);
#endif	//DEBUG
		seg_quality = 0;
			/*	if(gpscount > MAX_GPS_COUNT/5){
					i-=MAX_GPS_COUNT/5;
				}
*/				gpscount = 0;
				start_at=i;

			}
			gps_pts[gpscount].latitude = mm_request->gps_pts[i].latitude;
			gps_pts[gpscount].longitude = mm_request->gps_pts[i].longitude;
			gps_pts[gpscount].heading = mm_request->gps_pts[i].heading;
			gps_pts[gpscount].speed = mm_request->gps_pts[i].speed;
			gps_pts[gpscount].index = i-start_at;
			gpscount++;
			pts_skipped = 0;
			vzd_gps = 0;
			hed_last = mm_request->gps_pts[i].heading;
			hed_dif = 0;
		}
		else
		{
			pts_skipped++;
		}
	}

/*	mm_request_recursive.gps_pts = gps_pts;
	mm_request_recursive.gps_pts_size = gpscount;
	mm_response_recursive.map_pts = mm_pts;

	result = mm_map_match_recursive(ws, &mm_request_recursive, &mm_response_recursive);


		for (i = mm_request->gps_pts_size - 1; i >= 0; i--)		//set from last to first point
		{
			mm_response->map_pts[i].latitude = mm_response_recursive.map_pts[i].latitude;
			mm_response->map_pts[i].longitude = mm_response_recursive.map_pts[i].longitude;
		}

*/
#ifdef DEBUG
	fputs(vypis_gps,fw);
	fprintf(fw,"]\n");
	fflush(fw);
	fclose (fw);
	fputs(vypis_mm,fw_mm);
	fprintf(fw_mm,"]\n");
	fflush(fw_mm);
	fclose (fw_mm);
	fputs(vypis_r,fr_mm);
	fprintf(fr_mm,"]\n");
	fflush(fr_mm);
	fclose (fr_mm);
	//printf("quality %f%% %i z %i \n",(1.0-((double)mm_quality/mm_request->gps_pts_size))*100,mm_quality,mm_request->gps_pts_size);

#endif	//DEBUG
	if(segments_count)
		mm_response->quality=(1-((double)bad_segments/segments_count)) * 100;
	else
		mm_response->quality=100;
#ifdef DEBUG

	printf("quality segments %i%%  \n",mm_response->quality);

#endif	//DEBUG
	free(gps_pts);
	free(mm_pts);
		//free(gps_indexes);

	return MM_OK;

	//return mm_map_match_recursive(ws, mm_request, mm_response);
}

int find_reasonable_count(MM_WS * ws, IMM_MAP_SOURCE map_source, MM_GPS_PT *gps_pts, int gps_count)	///HERE///
{
	int i, rows_count;
	double max_dis_lon;
	char query[4096];
	const char *query_ptr = query;
	xdb_result_t *xdb_result = NULL;
	xdb_t*	xdb;
	int count = gps_count;
	double min_long, min_lat, max_long, max_lat;

	if (gps_count==0)
		return 0;

	count *= 2;

	do{
		count /= 2;

		min_long 	= DBL_MAX;
		min_lat 	= DBL_MAX;
		max_long 	= -DBL_MAX;
		max_lat 	= -DBL_MAX;

		//FIND BOUNDARY VALUES
		for (i = 0; i < count; i++)		// count boundary of points
		{
			if(i==0){
				max_dis_lon = GPS_DISCREPANCY*(cos( gps_pts[i].latitude * (0.0174533)));
			}
			if (gps_pts[i].latitude > max_lat)
				max_lat = gps_pts[i].latitude;
			if (gps_pts[i].latitude < min_lat)
				min_lat = gps_pts[i].latitude;
			if (gps_pts[i].longitude > max_long)
				max_long = gps_pts[i].longitude;
			if (gps_pts[i].longitude < min_long)
				min_long = gps_pts[i].longitude;
		}

		///HERE///
		if (map_source == IMM_OSM){
			sprintf(query, "SELECT id FROM %s WHERE MBRIntersects(PolygonFromText(\'POLYGON((%lf %lf,%lf %lf,%lf %lf,%lf %lf,%lf %lf))\'), %s) LIMIT %i",
					OSM_TABLE,
					min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
					max_long + max_dis_lon, max_lat + GPS_DISCREPANCY,	// top right
					max_long + max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom right
					min_long - max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom left
					min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
					NEW_LINE_ATTRIBUTE,
					MAX_DB_LINES);
		}
		else if (map_source == IMM_HERE)
		{
			sprintf(query, "SELECT id FROM %s WHERE MBRIntersects(PolygonFromText(\'POLYGON((%lf %lf,%lf %lf,%lf %lf,%lf %lf,%lf %lf))\'), %s) LIMIT %i",
					HERE_TABLE,
					min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
					max_long + max_dis_lon, max_lat + GPS_DISCREPANCY,	// top right
					max_long + max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom right
					min_long - max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom left
					min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
					NEW_LINE_ATTRIBUTE,
					MAX_DB_LINES);
		}
		else
		{
			//TODO
		}

		xdb = &(ws->xdb);

		xdb_result = NULL;
		xdb_result = xdb_query(xdb, query_ptr);

		if (xdb_result == NULL)
		{
			return -1;
		}

		rows_count = xdb_result->row_count;

		xdb_free_result(xdb_result);
	}
	while(rows_count == MAX_DB_LINES);

	return count;
}

int find_reasonable_count_2(MM_WS * ws, IMM_MAP_SOURCE map_source, MM_DATA_STRUCT *gps_pts, int gps_count)	///HERE///
{
	int i, rows_count;
	double max_dis_lon;
	char query[4096];
	const char *query_ptr = query;
	xdb_result_t *xdb_result = NULL;
	xdb_t*	xdb;
	int count = gps_count;
	double min_long, min_lat, max_long, max_lat;

	if (gps_count==0)
		return 0;

	count *= 2;

	do{
		count /= 2;

		min_long 	= DBL_MAX;
		min_lat 	= DBL_MAX;
		max_long 	= -DBL_MAX;
		max_lat 	= -DBL_MAX;

		//FIND BOUNDARY VALUES
		for (i = 0; i < count; i++)		// count boundary of points
		{
			if(i==0){
				max_dis_lon = GPS_DISCREPANCY*(cos( gps_pts[i].latitude * (0.0174533)));
			}
			if (gps_pts[i].latitude > max_lat)
				max_lat = gps_pts[i].latitude;
			if (gps_pts[i].latitude < min_lat)
				min_lat = gps_pts[i].latitude;
			if (gps_pts[i].longitude > max_long)
				max_long = gps_pts[i].longitude;
			if (gps_pts[i].longitude < min_long)
				min_long = gps_pts[i].longitude;
		}

		///HERE///
		if (map_source == IMM_OSM){
			sprintf(query, "SELECT id FROM %s WHERE MBRIntersects(PolygonFromText(\'POLYGON((%lf %lf,%lf %lf,%lf %lf,%lf %lf,%lf %lf))\'), %s) LIMIT %i",
					OSM_TABLE,
					min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
					max_long + max_dis_lon, max_lat + GPS_DISCREPANCY,	// top right
					max_long + max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom right
					min_long - max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom left
					min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
					NEW_LINE_ATTRIBUTE,
					MAX_DB_LINES);
		}
		else if (map_source == IMM_HERE)
		{
			sprintf(query, "SELECT id FROM %s WHERE MBRIntersects(PolygonFromText(\'POLYGON((%lf %lf,%lf %lf,%lf %lf,%lf %lf,%lf %lf))\'), %s) LIMIT %i",
					HERE_TABLE,
					min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
					max_long + max_dis_lon, max_lat + GPS_DISCREPANCY,	// top right
					max_long + max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom right
					min_long - max_dis_lon, min_lat - GPS_DISCREPANCY,	// bottom left
					min_long - max_dis_lon, max_lat + GPS_DISCREPANCY,	// top left
					NEW_LINE_ATTRIBUTE,
					MAX_DB_LINES);
		}
		else
		{
			//TODO
		}

		xdb = &(ws->xdb);

		xdb_result = NULL;
		xdb_result = xdb_query(xdb, query_ptr);

		if (xdb_result == NULL)
		{
			return -1;
		}

		rows_count = xdb_result->row_count;

		xdb_free_result(xdb_result);
	}
	while(rows_count == MAX_DB_LINES);

	return count;
}


double difangdeg(double x, double y) {
double arg;

if (x < y) {
arg = fmod(x - y, C360);
} else {
arg = fmod(y - x, C360);
}

if (arg < 0)
arg = arg + C360;
if (arg > 180)
arg = arg - C360;

return abs(arg);
}

/*
* urcuje jestli jsou dva uhly stejne v ramci tolerance
*/
int HeadingDifference(MM_GPS_PT pt1, MM_GPS_PT pt2) {
	int result ;

	if(pt1.speed < MIN_SPEED_FOR_HEADING || pt2.speed < MIN_SPEED_FOR_HEADING)
		return 0;

	result = difangdeg(pt1.heading, pt2.heading);

	if (result > HEADING_TOLERANCE) {
			return result;
	}
	return 0;

}
int IsPattern2(MM_GPS_PT* points, int no_controlled_points, int *index1, int *index2) {
	int i, j;
	int Max_Heading = 0;
	int Actual_Heading = 0;
	//for (i = 0; i <= no_controlled_points - 1; i++) {// uprava JH
	i=0;
		for (j = i + 1; j <= no_controlled_points - 1; j++) {
			Actual_Heading = HeadingDifference(points[i], points[j]);
			if (Actual_Heading > Max_Heading) {
				Max_Heading = Actual_Heading;
				*index1 = i;
				*index2 = j;
				if(Max_Heading>MAX_ANGLE_FOR_PAT){
									break;
				}
			}
			if(vzd_by_degree(points[i].longitude,points[i].latitude,points[j].longitude,points[j].latitude)>MAX_DIST_FOR_PAT)
				break;
		}
	return Max_Heading;
	//}
	//if (Max_Heading) {
		//return TRUE;
	//} else {
		//return FALSE;
	//}
}

void findPatterns(MM_REQUEST* mm_request_2)
{
	int k;
	int index1, index2, last_max_uhel, last_index1, last_index2, uhel;


	//double dist_from_last_pat = 0;
	last_max_uhel = 0;
	last_index2 = 0;

	for (k=0;k<mm_request_2->gps_pts_size - 1;k++)
	{
		uhel = IsPattern2(&mm_request_2->gps_pts[k],MIN(STEP,mm_request_2->gps_pts_size - k), &index1, &index2);
		if(uhel){

			if(uhel>last_max_uhel && k <= last_index2 && last_index2){
					mm_request_2->gps_pts[last_index1].fix_status=0;
					mm_request_2->gps_pts[last_index2].fix_status=0;
			}
			if(uhel<=last_max_uhel && k <= last_index2 && last_index2){
				;
			}else{
				if(last_index2 && k > last_index2){
					if(vzd_by_degree(mm_request_2->gps_pts[last_index2].longitude,mm_request_2->gps_pts[last_index2].latitude,
							mm_request_2->gps_pts[k+index1].longitude,mm_request_2->gps_pts[k+index1].latitude)>MIN_PAT_DIST){
										mm_request_2->gps_pts[k+index1].fix_status=10;
										mm_request_2->gps_pts[k+index2].fix_status=20;
										last_index1 = index1+k;
										last_index2 = index2+k;
										last_max_uhel = uhel;
					}
				}else{
					mm_request_2->gps_pts[k+index1].fix_status=10;
					mm_request_2->gps_pts[k+index2].fix_status=20;
					last_index1 = index1+k;
					last_index2 = index2+k;
					last_max_uhel = uhel;
				}
				if(last_max_uhel>MAX_ANGLE_FOR_PAT && k< last_index2+1){
					k=last_index2+1;
				}
			}

		}

	}

}

int FindLRPattern(int *index_pattern_child_left,  int* index_pattern_child_right, MM_REQUEST* mm_request_2,int orientation)
{
	int i;

	if(orientation == 0){

	for(i=(*index_pattern_child_right) + 1; i< mm_request_2->gps_pts_size -1;i++)
	{
		if(mm_request_2->gps_pts[i].fix_status == 10) // first point of pattern
		{
			*index_pattern_child_left = i;
		}
		if(mm_request_2->gps_pts[i].fix_status == 20) // first point of pattern
		{
			*index_pattern_child_right = i;
			return 0;
		}
	}
	}
	else
	{

		if(*index_pattern_child_right == -1)
			*index_pattern_child_right =  mm_request_2->gps_pts_size;
		for(i=(*index_pattern_child_right-1); i>= 1;i--)
		{
			if(mm_request_2->gps_pts[i].fix_status == 20) // first point of pattern
			{
				*index_pattern_child_left = i;
			}
			if(mm_request_2->gps_pts[i].fix_status == 10) // first point of pattern
			{
				*index_pattern_child_right = i;
				return 0;
			}
		}

	}

	return 1;

}

int difangdegSup(int heading1, int heading2) {
int arg;

if (heading1 < heading2) {
arg = heading2 - heading1;
} else {
arg = heading1 - heading2;
}

if (arg > 180) {
arg = 360 - arg;
}

//printf("%i\n", arg);
return arg; }

// x prohozeny za y !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
int angleBetweenLines(double p1_x, double p1_y, double p2_x, double p2_y,
double p3_x, double p3_y, double p4_x, double p4_y) {
	double result;
double degree_coef = cos(p1_x * (0.0174533));
double result1, result2;
//result1 = 90 - (atan2(p2_y - p1_y, (p2_x - p1_x) * degree_coef) * (180.0 / M_PI));
//result2 = 90 - (atan2(p4_y - p3_y, (p4_x - p3_x) * degree_coef) * (180.0 / M_PI));

result1 = (atan2(p2_y - p1_y, (p2_x - p1_x) / degree_coef) * (180.0 / M_PI));
result2 = (atan2(p4_y - p3_y, (p4_x - p3_x) / degree_coef) * (180.0 / M_PI));

/*if(result1 < 0) {
result1 = 360 + result1;
}

if(result2 < 0) {
result2 = 360 + result2;
}*/
result = fabs(result1-result2);
if(result > 180) {
result = 360 - result;
}

result = abs(result);

/*if (result < 180) {
result = 360 - result;
}*/

//printf("%lf\n", result);

return result; }

int angleBetweenLineAndPoint(double p1_x, double p1_y, double p2_x, double p2_y,int heading) {
	double result;
double degree_coef = cos(p1_y * (0.0174533));
double result1, result2;
//result1 = 90 - (atan2(p2_y - p1_y, (p2_x - p1_x) * degree_coef) * (180.0 / M_PI));
//result2 = 90 - (atan2(p4_y - p3_y, (p4_x - p3_x) * degree_coef) * (180.0 / M_PI));

result1 = (atan2(p2_x - p1_x, (p2_y - p1_y) / degree_coef) * (180.0 / M_PI));
result2 = heading;

/*if(result1 < 0) {
result1 = 360 + result1;
}

if(result2 < 0) {
result2 = 360 + result2;
}*/
result = fabs(result1-result2);
if(result > 180) {
result = 360 - result;
}

result = abs(result);

/*if (result < 180) {
result = 360 - result;
}*/

//printf("%lf\n", result);

return result; }
int IsAnglesSame(int ap1, int ap2, double p1_x, double p1_y, double p2_x, double p2_y,
double p3_x, double p3_y, double p4_x, double p4_y) {

double degree_coef = cos(p1_x * (0.0174533));
double result1, result2;
//result1 = 90 - (atan2(p2_y - p1_y, (p2_x - p1_x) * degree_coef) * (180.0 / M_PI));
//result2 = 90 - (atan2(p4_y - p3_y, (p4_x - p3_x) * degree_coef) * (180.0 / M_PI));

result1 = (atan2(p2_y - p1_y, (p2_x - p1_x) / degree_coef) * (180.0 / M_PI));
result2 = (atan2(p4_y - p3_y, (p4_x - p3_x) / degree_coef) * (180.0 / M_PI));

result1 = fabs(result1-ap1);
if(result1 > 180) {
result1 = 360 - result1;
}
result2 = fabs(result2-ap2);
if(result2 > 180) {
result2 = 360 - result2;
}

if(abs(result1)<ANGLE_TOLERANCE_NEARBY && abs(result2)<ANGLE_TOLERANCE_NEARBY)
	return TRUE;

return FALSE;
}

BOOL IsAngleSimilar(int a1, int a2) {
//int result = difangdegSup(a1, a2); //!!!
int result = abs(a1-a2);
if (result <= ANGLE_TOLERANCE_TOTAL) {
return TRUE;
} else {
return FALSE;
}
}



BOOL IsDistAccept(double dist_gps, double total_dist_route)
{
if(fabs(dist_gps - total_dist_route)<10)
return TRUE;
//if((fabs(dist_gps - total_dist_route))/((dist_gps + total_dist_route)/2)>0.2)
//return TRUE;

if (!total_dist_route)
return FALSE;
if(((fabs(dist_gps - total_dist_route))/total_dist_route) > 0.25)
return FALSE;
else
return TRUE;

//return FALSE;
}

int get_line_intersection(double a1_x, double a1_y, double a2_x, double a2_y,double b1_x, double b1_y, double b2_x, double b2_y,double *res_x, double *res_y)
{
double f1 = -(a2_y - a1_y) / (a1_x - a2_x);
double f2 = -(b2_y - b1_y) / (b1_x - b2_x);
double g1 = a1_y - f1 * a1_x;
double g2 = b1_y - f2 * b1_x;

// rovnob�ky -> ��dn� pr�se��k
if(f1 == f2)
return 0;

double inter_x, inter_y;
if (a1_x == a2_x) {
inter_x = a1_x;
inter_y = f2*a1_x + g2;
}
else if (b1_x == b2_x){
inter_x = b1_x;
inter_y = f1*b1_x + g1;
}
else
{
inter_x = (g2-g1)/(f1 -f2);
inter_y = f1 * inter_x + g1;
}

*res_x = inter_x;
*res_y = inter_y;

return 1;
}


int findCrossings(MM_GPS_PT* gps1, MM_GPS_PT* gps2,											// pt1 & pt2 in
		double edge1_pt1_lat, double edge1_pt1_lon, double edge1_pt2_lat, double edge1_pt2_lon,	// edge1 in
		double edge2_pt1_lat, double edge2_pt1_lon, double edge2_pt2_lat, double edge2_pt2_lon,	// edge2 in
		double *edge_cross_lat, double *edge_cross_lon, double *pt_cross_lat, double *pt_cross_lon)	// out points
{
	int crossed;

    double degree_coef = cos(gps1->latitude * (0.0174533));

    //degree_coef* lon


	crossed = get_line_intersection(edge1_pt1_lon, edge1_pt1_lat, edge1_pt2_lon, edge1_pt2_lat,
			edge2_pt1_lon, edge2_pt1_lat, edge2_pt2_lon, edge2_pt2_lat, edge_cross_lon, edge_cross_lat);
	if (!crossed)
		return crossed;	// No crossing

	double x, plus_lon1,plus_lon2,plus_lat1,plus_lat2;
	//x = 0.0007;
	x = 1;
	plus_lon1 = x * sin(gps1->heading*M_PI/180) ;
	plus_lon2 = x * sin(gps2->heading*M_PI/180) ;
	plus_lat1 = x * cos(gps1->heading*M_PI/180) * degree_coef;
	plus_lat2 = x * cos(gps2->heading*M_PI/180) * degree_coef;



	crossed = get_line_intersection(gps1->longitude, gps1->latitude, gps1->longitude + plus_lon1, gps1->latitude + plus_lat1,gps2->longitude, gps2->latitude, gps2->longitude + plus_lon2, gps2->latitude + plus_lat2, pt_cross_lon, pt_cross_lat);
	if (!crossed)
		return crossed;	// No crossing
#ifdef DEBUG
		printf("gps [[%lf,%lf],[%lf,%lf],[%lf,%lf],[%lf,%lf]]\n", gps1->latitude,gps1->longitude,  gps1->latitude + plus_lat1,
				gps1->longitude + plus_lon1, gps2->latitude,gps2->longitude, gps2->latitude + plus_lat2, gps2->longitude + plus_lon2);
		printf("gps [[%lf,%lf]]\n",*pt_cross_lat, *pt_cross_lon);

#endif	//DEBUG
    return crossed;
}


int MoveToLineOnOffset(double lat_offset, double lon_offset, MM_GPS_PT* req_left, MM_GPS_PT* req_right, MM_MAP_PT* res_left, MM_MAP_PT* res_right,MM_POINT_LL* route_line,int route_line_number, int orientation)
{
	double res_lon, res_lat, distance_left, shortest_distance_left=DBL_MAX,distance_right, shortest_distance_right=DBL_MAX;
	double degree_coef = cos(req_left->latitude * (0.0174533));
	int i;

/*	res_left->latitude = req_left.latitude + lat_offset;
	res_left->longitude = req_left.longitude + lon_offset;
	res_right->latitude = req_right.latitude + lat_offset;
	res_right->longitude = req_right.longitude + lon_offset;
*/
	req_left->fix_status += 1; // no more needed, result stored in response
	req_right->fix_status += 1;

    for(i = 0; i<route_line_number-2; i++)
    {
      pointOnSegment((req_left->longitude + lon_offset) * degree_coef, req_left->latitude + lat_offset, route_line[i].x*degree_coef, route_line[i].y, route_line[i+1].x*degree_coef, route_line[i+1].y, &res_lon, &res_lat);
       res_lon = res_lon / degree_coef;

        distance_left = vzd_by_degree(req_left->longitude + lon_offset,req_left->latitude + lat_offset, res_lon, res_lat);
        if(distance_left < shortest_distance_left) {
            // Found a better solution, use this one
        	res_left->longitude = res_lon;
            res_left->latitude = res_lat;
            shortest_distance_left = distance_left;
            if(!orientation)
            	res_left->match_quality = i+1; //index +1 because 0 is reserved for non matched points
            else
            	res_left->match_quality = route_line_number -(i+1);
            //res_left->id = 999;
        }
    }
    for(i = 0; i<route_line_number-2; i++)
    {
        pointOnSegment((req_right->longitude + lon_offset) * degree_coef, req_right->latitude + lat_offset, route_line[i].x*degree_coef, route_line[i].y, route_line[i+1].x*degree_coef, route_line[i+1].y, &res_lon, &res_lat);
        res_lon = res_lon / degree_coef;

         distance_right= vzd_by_degree(req_right->longitude + lon_offset,req_right->latitude + lat_offset, res_lon, res_lat);
         if(distance_right < shortest_distance_right) {
             // Found a better solution, use this one
         	 res_right->longitude = res_lon;
             res_right->latitude = res_lat;
             shortest_distance_right = distance_right;
         	if(!orientation)
        		res_right->match_quality = i+1; //index +1 because 0 is reserved for non matched points
        	            else
        	      res_right->match_quality = route_line_number -(i+1);
         	res_right->id = 999;
         }
   }


#ifdef DEBUG
		printf("gps moved by offset [[%lf,%lf],[%lf,%lf]]\n",res_left->latitude,res_left->longitude,res_right->latitude,res_right->longitude );
#endif	//DEBUG
return 0;
}


int MoveToLineOnOffset_v2(double lat_offset, double lon_offset, MM_GPS_PT* req_left, MM_GPS_PT* req_right, MM_POINT_LL* res_left, MM_POINT_LL* res_right, int* left_line_index, int* right_line_index, MM_POINT_LL* route_line,int route_line_number,int index_best_angle_left, int p_l, int p_r)
{
	double res_lon, res_lat, distance_left, shortest_distance_left=DBL_MAX,distance_right, shortest_distance_right=DBL_MAX;
	double degree_coef = cos(req_left->latitude * (0.0174533));
	int i, result = 1, r_angle, l_angle;

	//req_left->fix_status += 1; // no more needed, result stored in response
	//req_right->fix_status += 1;


	if(index_best_angle_left >2)
		index_best_angle_left -=2;

    for(i = index_best_angle_left; i<route_line_number-2; i++)
    {
      pointOnSegment((req_left->longitude + lon_offset) * degree_coef, req_left->latitude + lat_offset, route_line[i].x*degree_coef, route_line[i].y, route_line[i+1].x*degree_coef, route_line[i+1].y, &res_lon, &res_lat);
       res_lon = res_lon / degree_coef;

       distance_left = vzd_by_degree(req_left->longitude + lon_offset,req_left->latitude + lat_offset, res_lon, res_lat);
       if(distance_left < shortest_distance_left) {
    	   // Found a better solution, use this one
    	   l_angle = angleBetweenLineAndPoint(route_line[i].x, route_line[i].y,route_line[i+1].x, route_line[i+1].y,p_l);
    	   if(l_angle > ANGLE_TOLERANCE_NEARBY)
    		   result = 1;
    	   else{
    		   res_left->x = res_lon;
    		   res_left->y = res_lat;
    		   shortest_distance_left = distance_left;
    		   *left_line_index = i+1; //index +1 because 0 is reserved for non matched points
    		   result = 0;
    	   }

    	   //res_left->id = 999;
       }
    }
    for(i = index_best_angle_left; i<route_line_number-2; i++)
    {
    	pointOnSegment((req_right->longitude + lon_offset) * degree_coef, req_right->latitude + lat_offset, route_line[i].x*degree_coef, route_line[i].y, route_line[i+1].x*degree_coef, route_line[i+1].y, &res_lon, &res_lat);
    	res_lon = res_lon / degree_coef;

    	distance_right= vzd_by_degree(req_right->longitude + lon_offset,req_right->latitude + lat_offset, res_lon, res_lat);
    	if(distance_right < shortest_distance_right) {
    		// Found a better solution, use this one
    		r_angle = angleBetweenLineAndPoint(route_line[i].x, route_line[i].y,route_line[i+1].x, route_line[i+1].y,p_r);
    		if(r_angle > ANGLE_TOLERANCE_NEARBY)
    			result = 1;
    		else{
    			res_right->x = res_lon;
    			res_right->y = res_lat;
    			shortest_distance_right = distance_right;

    			*right_line_index = i+1; //index +1 because 0 is reserved for non matched points
    			result = 0;
    	}
    	}
    }


#ifdef DEBUG
		printf("gps moved by offset [[%lf,%lf],[%lf,%lf]]\n",res_left->y,res_left->x,res_right->y,res_right->x );
		printf("indexy %i,%i\n",*right_line_index, *left_line_index);

#endif	//DEBUG
return result;
}


int OtocSmer(int alpha){
	alpha = alpha + 180;
	if(alpha > 360)
		alpha -= 360;
	return alpha;
}

int OtocOrientaciLinie(MM_POINT_LL* route_line,int route_line_number){
	MM_POINT_LL* line_arr, tmp;
	int i;
	line_arr = route_line;
	for (i = 0; i< route_line_number/2; i++){
		tmp = line_arr[i];
		line_arr[i] = line_arr[(route_line_number-1)-i];
		line_arr[(route_line_number-1)-i] = tmp;
  }
	return 0;
}

int FindPatternMatchR(MM_REQUEST* mm_request_2, MM_RESPONSE* mm_response_2, MM_POINT_LL* route_line,int route_line_number, int orientation)
{

	BOOL match_found = FALSE;

	int i,index_pattern_child_left, index_pattern_child_right, index_pattern_child_right_old,  index_line,index_line_inner;
	double best_angle_diff;
	int index_best_angle_left,index_best_angle_right,old_index_pattern_child_right;
	int floating_counter = 0;
	int line_arr_length = 0;
	double linesAngle, dist_gps, total_dist_route;
	MM_POINT_LL* line_arr;//, last_matched;
	//MM_GPS_PT* pattern_arr_all;
	MM_GPS_PT* pattern_arr;
	/*pattern_arr = (MM_GPS_PT*) calloc(10000, sizeof(MM_GPS_PT));*/
	double d_pattern_linepoint_right,d_pattern_linepoint_left;
	double edge_cross_lat, edge_cross_lon,pt_cross_lat, pt_cross_lon;
	int matched = 0;
	int p_l, p_r;

	line_arr_length = route_line_number;
	line_arr = route_line;
	pattern_arr = mm_request_2->gps_pts;
	best_angle_diff = -1;
	index_best_angle_right = 0;
	index_pattern_child_right_old = -1;
	total_dist_route = 0.0;
	dist_gps = 0.0;

	index_pattern_child_right = -1; // incremented if next method
	if(FindLRPattern(&index_pattern_child_left, &index_pattern_child_right,mm_request_2,orientation))
		return matched;

	OtocOrientaciLinie(line_arr,line_arr_length);


	for (index_line = 1; index_line < line_arr_length-2; index_line++) {


		if(	index_best_angle_right >0 && (index_line + 1) > index_best_angle_right+1 )
			total_dist_route+=vzd_by_degree(line_arr[index_line - 1].x,line_arr[index_line - 1].y,line_arr[index_line ].x,line_arr[index_line ].y);

		double dist_pat;
		for (index_line_inner = index_line;	/*index_line_inner < (LINES_FOR_PAT_FINDING + index_line)*/ ((dist_pat = vzd_by_degree(line_arr[index_line].x,line_arr[index_line].y,line_arr[index_line_inner].x,line_arr[index_line_inner].y))	 < DIST_FOR_PAT_FINDING) &&
		index_line_inner < (line_arr_length-1);index_line_inner++) {
		//porovnani uhlu mezi dvema body patternu a dvema liniemi

		// 1. spocitam uhel mezi dvema liniemi
		//spocita uhel mezi dvema liniemi
		 linesAngle = angleBetweenLines(
				line_arr[index_line - 1].y,
				line_arr[index_line - 1].x,
				line_arr[index_line].y,
				line_arr[index_line].x,
				line_arr[index_line_inner].y,
				line_arr[index_line_inner].x,
				line_arr[index_line_inner + 1].y,
				line_arr[index_line_inner + 1].x); //tady to asi na konci pretece

		//TESTOVACI VYPIS
		/*if (index_line_inner > 100 && index_line_inner < 115) {
			printf("uhel mezi %i, %i a %i, %i: %lf\n", (index_line - 1),
					index_line, index_line_inner, (index_line_inner + 1),
					linesAngle);
		}*/
		/*if (index_line == 105 && index_line_inner == 112) {
			printf("\n");
		}*/

		// 2. spocitam uhel mezi dvema patterny
		p_l = orientation ? OtocSmer(pattern_arr[index_pattern_child_left].heading) : pattern_arr[index_pattern_child_left].heading;
		p_r = orientation ? OtocSmer(pattern_arr[index_pattern_child_right].heading) : pattern_arr[index_pattern_child_right].heading;
		int difang = difangdegSup(p_l,p_r);

		//int difang = difangdegSup(pattern_arr[index_pattern_child_left].heading,pattern_arr[index_pattern_child_right].heading);

		/*if(difang > 90) {
		 difang = 180 - difang;
		 }*/
		int anglesOK = IsAnglesSame(p_l,p_r,line_arr[index_line - 1].y,
						line_arr[index_line - 1].x,
						line_arr[index_line].y,
						line_arr[index_line].x,
						line_arr[index_line_inner].y,
						line_arr[index_line_inner].x,
						line_arr[index_line_inner + 1].y,
						line_arr[index_line_inner + 1].x);
		/*int anglesOK = IsAnglesSame(pattern_arr[index_pattern_child_left].heading,pattern_arr[index_pattern_child_right].heading,line_arr[index_line - 1].y,
				line_arr[index_line - 1].x,
				line_arr[index_line].y,
				line_arr[index_line].x,
				line_arr[index_line_inner].y,
				line_arr[index_line_inner].x,
				line_arr[index_line_inner + 1].y,
				line_arr[index_line_inner + 1].x);*/

		int distOK = IsDistAccept(dist_gps,total_dist_route);
		// porovnam uhly
		if (IsAngleSimilar((int) linesAngle, difang) && anglesOK && distOK && index_pattern_child_right_old < index_line) {
			// slozitejsi varianta s porovnanim vzdalenosti patternu od potencialne matchnuteho bodu na linii

			d_pattern_linepoint_left = vzd_by_degree(line_arr[index_line].x,line_arr[index_line].y,pattern_arr[index_pattern_child_left].longitude,pattern_arr[index_pattern_child_left].latitude);
			d_pattern_linepoint_right = vzd_by_degree(line_arr[index_line_inner].x,line_arr[index_line_inner].y,pattern_arr[index_pattern_child_right].longitude,pattern_arr[index_pattern_child_right].latitude);

			if (d_pattern_linepoint_right <= DISTANCE_TOLERANCE && d_pattern_linepoint_left <= DISTANCE_TOLERANCE) {
				int best = abs(linesAngle);//int best = abs(linesAngle - difang);
				if (best_angle_diff < best) {
					best_angle_diff = best;
					index_best_angle_right = index_line_inner;
					index_best_angle_left = index_line;
					total_dist_route = 0.0;
					//nastavim match_found na TRUE, protoze jsme nasli shodu uhlu
  					match_found = TRUE;
				}



				//break;
			}
			// slozitejsi varianta KONEC
		} else {
			// nastavim na 999 pokud jsem nenasel shodu
			mm_response_2->map_pts[index_pattern_child_left].latitude=999.0;
			mm_response_2->map_pts[index_pattern_child_left].longitude=999.0;
			mm_response_2->map_pts[index_pattern_child_right].latitude=999.0;
			mm_response_2->map_pts[index_pattern_child_right].longitude=999.0;
		}
	}



	if(match_found)
		floating_counter++;
	// index patternu posunuji pouze kdyz jsme je namatchovali
	if (match_found == TRUE && floating_counter == FLOATING_COUNTER) {



		findCrossings(&pattern_arr[index_pattern_child_left],&pattern_arr[index_pattern_child_right],
				line_arr[index_best_angle_left-1].y,line_arr[index_best_angle_left-1].x,line_arr[index_best_angle_left].y,line_arr[index_best_angle_left].x,
				line_arr[index_best_angle_right].y,line_arr[index_best_angle_right].x,line_arr[index_best_angle_right+1].y,line_arr[index_best_angle_right+1].x,
				&edge_cross_lat, &edge_cross_lon, &pt_cross_lat, &pt_cross_lon);



#ifdef DEBUG
		printf("line [[%lf,%lf],[%lf,%lf],[%lf,%lf],[%lf,%lf]]\n",line_arr[index_best_angle_left-1].y,line_arr[index_best_angle_left-1].x,line_arr[index_best_angle_left].y,line_arr[index_best_angle_left].x,
				line_arr[index_best_angle_right].y,line_arr[index_best_angle_right].x,line_arr[index_best_angle_right+1].y,line_arr[index_best_angle_right+1].x);
		printf("line [[%lf,%lf]]\n",edge_cross_lat, edge_cross_lon);

#endif	//DEBUG

		// souradnice patternu nastavime na souradnice bodu z linie
		/*mm_response_2->map_pts[index_pattern_child_left].latitude=line_arr[index_best_angle_left].y;
		mm_response_2->map_pts[index_pattern_child_left].longitude=line_arr[index_best_angle_left].x;
		mm_response_2->map_pts[index_pattern_child_right].latitude=line_arr[index_best_angle_right].y;
		mm_response_2->map_pts[index_pattern_child_right].longitude=line_arr[index_best_angle_right].x;
*/

		MoveToLineOnOffset(edge_cross_lat - pt_cross_lat, edge_cross_lon - pt_cross_lon,
				&mm_request_2->gps_pts[index_pattern_child_left],&mm_request_2->gps_pts[index_pattern_child_right],
				&(mm_response_2->map_pts[index_pattern_child_left]),&(mm_response_2->map_pts[index_pattern_child_right]),
				route_line,route_line_number,orientation);


		matched++;


				// musim nastavit index_line na hodnotu index_best_angle_tmp
		// duvod? v hledani dalsi shody musim pokracovat, kde jsem skoncil
		//index_line = index_best_angle_tmp;

		old_index_pattern_child_right=index_pattern_child_right;
		if(FindLRPattern(&index_pattern_child_left, &index_pattern_child_right,mm_request_2,orientation))
					return matched;

		dist_gps = 0.0;
		for(i=0; i<(index_pattern_child_left - old_index_pattern_child_right);i++)
		{
			dist_gps+=vzd_by_degree(pattern_arr[old_index_pattern_child_right+i].longitude,pattern_arr[old_index_pattern_child_right+i].latitude,pattern_arr[old_index_pattern_child_right+i+1].longitude,pattern_arr[old_index_pattern_child_right+i+1].latitude);
		}


		//pro pocitani nejlepsiho uhlu
		best_angle_diff = 0;
		//index_best_angle_right = -1;

		// pripravim pro dalsi prohledavani
		match_found = FALSE;
	}

	if(floating_counter == FLOATING_COUNTER) {
		floating_counter = 0;
	}

	}
	return matched;

}


int FindPatternMatchR_v2(MM_REQUEST* mm_request_2, MM_POINT_LL* route_line,int route_line_number, MM_PATT_CANDIDATES* pat_candidates, int patt_count)
{

	BOOL match_found = FALSE;

	int index_line,index_line_inner;
	double best_angle_diff;
	int index_best_angle_left,index_best_angle_right;
	int floating_counter = 0;
	int line_arr_length = 0;
	double linesAngle;
	MM_POINT_LL* line_arr;
	//MM_GPS_PT* pattern_arr_all;
	MM_GPS_PT* pattern_arr;
	/*pattern_arr = (MM_GPS_PT*) calloc(10000, sizeof(MM_GPS_PT));*/
	//double d_pattern_linepoint_right,d_pattern_linepoint_left;
	double edge_cross_lat, edge_cross_lon,pt_cross_lat, pt_cross_lon;
	int matched = 0, visited;
	int p_l,p_r;
	int min_dist_to_patt, no_move_start_patt;

	line_arr_length = route_line_number;
	line_arr = route_line;
	pattern_arr = mm_request_2->gps_pts;
	best_angle_diff = 360;
	index_best_angle_right = 0;


#ifdef DEBUG
	char d_temp_string[50000];
	int d_count;

	if(file_angles == NULL)
		file_angles = fopen("/home/adminuser/workspace/mmnew/Debug/src/angles.csv", "w");
	else
		file_angles = fopen("/home/adminuser/workspace/mmnew/Debug/src/angles.csv", "a");
#endif



	int active_pattern,active_candidate=0, start_line_for_pat=1,old_start_line_for_pat=-1;
	double  dist_to_pattern;
	int i_null;
	for(active_pattern = 0;active_pattern < patt_count; active_pattern++)
	{
#ifdef DEBUG
		d_act_pattern=active_pattern;
		sprintf(d_temp_string, "\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> segment no: %i <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", d_act_segment);
		fputs(d_temp_string,file_angles);
		sprintf(d_temp_string, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> pattern no in segment: %i <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", d_act_pattern);
		fputs(d_temp_string,file_angles);
		sprintf(d_temp_string, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  pattern no in total: %i <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", d_act_global_pattern);
		fputs(d_temp_string,file_angles);
		d_act_global_pattern++;
		sprintf(d_temp_string, ">>>>>>>>>>>>>>>>>>>>> pattern coords: [%lf,%lf],[%lf,%lf] <<<<<<<<<<<<<<<<<<<<<<\n",
				pattern_arr[pat_candidates[active_pattern].left_index].latitude,
				pattern_arr[pat_candidates[active_pattern].left_index].longitude,
				pattern_arr[pat_candidates[active_pattern].right_index].latitude,
				pattern_arr[pat_candidates[active_pattern].right_index].longitude);
		fputs(d_temp_string,file_angles);


		fputs("\nLine :   [",file_angles);
		for (d_count = 0 ; d_count < route_line_number; d_count++){
			//route_line
			sprintf(d_temp_string, "[%lf,%lf],",
					route_line[d_count].y,
					route_line[d_count].x);
			fputs(d_temp_string,file_angles);
		}
		fputs("]\n\n",file_angles);

#endif


		floating_counter = 0;
		active_candidate=0;
		best_angle_diff = 360;
		match_found = FALSE;
		visited = -1;
		// initiate candidate arrays
		for(i_null = 0; i_null < MAX_PAT_CANDIDATES; i_null++) {
			pat_candidates[active_pattern].left_candidate[i_null].x = -1;
			pat_candidates[active_pattern].left_candidate[i_null].y = -1;
			pat_candidates[active_pattern].right_candidate[i_null].x = -1;
			pat_candidates[active_pattern].right_candidate[i_null].y = -1;
		}

		old_start_line_for_pat=-1;

		min_dist_to_patt=INT_MAX;
		no_move_start_patt = 0;

		for (index_line = start_line_for_pat; index_line < line_arr_length; index_line++) {

			dist_to_pattern = vzd_by_degree(mm_request_2->gps_pts[pat_candidates[active_pattern].right_index].longitude,mm_request_2->gps_pts[pat_candidates[active_pattern].right_index].latitude,
					line_arr[index_line ].x,line_arr[index_line ].y);

			if(min_dist_to_patt>dist_to_pattern){
				min_dist_to_patt=dist_to_pattern;
				if( no_move_start_patt == 0)
					start_line_for_pat=index_line;
			}

			if(dist_to_pattern < DISTANCE_TOLERANCE && visited < 1)
			{

				visited = 0;
				double dist_pat;
				for (index_line_inner = index_line;	 (dist_pat = vzd_by_degree(line_arr[index_line].x,line_arr[index_line].y,line_arr[index_line_inner].x,line_arr[index_line_inner].y)) < DIST_FOR_PAT_FINDING && index_line_inner < line_arr_length-1;index_line_inner++)
				{
					//porovnani uhlu mezi dvema body patternu a dvema liniemi

					// 1. spocitam uhel mezi dvema liniemi
					//spocita uhel mezi dvema liniemi
					linesAngle = angleBetweenLines(
							line_arr[index_line - 1].y,
							line_arr[index_line - 1].x,
							line_arr[index_line].y,
							line_arr[index_line].x,
							line_arr[index_line_inner].y,
							line_arr[index_line_inner].x,
							line_arr[index_line_inner + 1].y,
							line_arr[index_line_inner + 1].x); //tady to asi na konci pretece


					// 2. spocitam uhel mezi dvema patterny
					p_l = pattern_arr[pat_candidates[active_pattern].left_index].heading;
					p_r = pattern_arr[pat_candidates[active_pattern].right_index].heading;
					int difang = difangdegSup(p_l,p_r);

					//int difang = difangdegSup(pattern_arr[index_pattern_child_left].heading,pattern_arr[index_pattern_child_right].heading);

					int anglesOK = IsAnglesSame(p_l,p_r,line_arr[index_line - 1].y,
							line_arr[index_line - 1].x,
							line_arr[index_line].y,
							line_arr[index_line].x,
							line_arr[index_line_inner].y,
							line_arr[index_line_inner].x,
							line_arr[index_line_inner + 1].y,
							line_arr[index_line_inner + 1].x);

					//int distOK = IsDistAccept(dist_gps,total_dist_route);
					// porovnam uhly

#ifdef DEBUG
		//if (d_act_segment == 2 && d_act_pattern == 1){
			sprintf(d_temp_string, "line angle: %i, pattern angle: %i\n", (int)linesAngle, difang);
			fputs(d_temp_string,file_angles);
			sprintf(d_temp_string, "[%lf,%lf],[%lf,%lf],[%lf,%lf],[%lf,%lf]\n",
					line_arr[index_line - 1].y,
					line_arr[index_line - 1].x,
					line_arr[index_line].y,
					line_arr[index_line].x,
					line_arr[index_line_inner].y,
					line_arr[index_line_inner].x,
					line_arr[index_line_inner + 1].y,
					line_arr[index_line_inner + 1].x);
			fputs(d_temp_string,file_angles);
		//}
#endif


		if (IsAngleSimilar((int) linesAngle, difang) && anglesOK) {// && distOK) {
			// slozitejsi varianta s porovnanim vzdalenosti patternu od potencialne matchnuteho bodu na linii

			if(old_start_line_for_pat==-1){
				if(start_line_for_pat>index_line)
					start_line_for_pat = index_line;
				old_start_line_for_pat=1;
				no_move_start_patt = 1;
			}
		/*	d_pattern_linepoint_left = vzd_by_degree(line_arr[index_line].x,line_arr[index_line].y,pattern_arr[index_pattern_child_left].longitude,pattern_arr[index_pattern_child_left].latitude);
			d_pattern_linepoint_right = vzd_by_degree(line_arr[index_line_inner].x,line_arr[index_line_inner].y,pattern_arr[index_pattern_child_right].longitude,pattern_arr[index_pattern_child_right].latitude);

			if (d_pattern_linepoint_right <= DISTANCE_TOLERANCE && d_pattern_linepoint_left <= DISTANCE_TOLERANCE) {*/
			//	int best = abs(linesAngle - difang);//int best = abs(linesAngle);

		/*
			dist1 = routeDistance(mm_request_2->gps_pts,pat_candidates[p].left_index, pat_candidates[p2].right_index,0);
			dist2 = routeDistance(mm_request_2->gps_pts,pat_candidates[p].right_index, pat_candidates[p2].left_index,0);
			dist_pat = (dist1+dist2) / 2;*/
			double dist_for_best = (distToLine(mm_request_2->gps_pts[pat_candidates[active_pattern].left_index].longitude,mm_request_2->gps_pts[pat_candidates[active_pattern].left_index].latitude,
					line_arr[index_line ].x,line_arr[index_line ].y,line_arr[index_line-1 ].x,line_arr[index_line-1 ].y)+
					distToLine(mm_request_2->gps_pts[pat_candidates[active_pattern].right_index].longitude,mm_request_2->gps_pts[pat_candidates[active_pattern].right_index].latitude,
					line_arr[index_line_inner ].x,line_arr[index_line_inner ].y,line_arr[index_line_inner+1 ].x,line_arr[index_line_inner+1 ].y))/2.0;
//sssssss
			double best = abs(linesAngle - difang) + (5*((dist_for_best+1)/DISTANCE_TOLERANCE)) + (10*(MAX(50,dist_for_best)/50));// added distance coef
			if (best_angle_diff > best) {//if (best_angle_diff < best) {


#ifdef DEBUG
			fputs("BEST ANGLE FOR PATTERN SO FAR<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n",file_angles);
			sprintf(d_temp_string, "best:%lf, prev:%lf, D1:%lf, D2:%lf\n", best, best_angle_diff,dist_to_pattern,dist_for_best);
			fputs(d_temp_string,file_angles);
#endif
			best_angle_diff = best;
			index_best_angle_right = index_line_inner;
			index_best_angle_left = index_line;

					//nastavim match_found na TRUE, protoze jsme nasli shodu uhlu
  					match_found = TRUE;
				/*}*/



				//break;
			}
			// slozitejsi varianta KONEC
		} /*else {
			// nastavim na 999 pokud jsem nenasel shodu
			mm_response_2->map_pts[index_pattern_child_left].latitude=999.0;
			mm_response_2->map_pts[index_pattern_child_left].longitude=999.0;
			mm_response_2->map_pts[index_pattern_child_right].latitude=999.0;
			mm_response_2->map_pts[index_pattern_child_right].longitude=999.0;
		}*/
	}
	if(match_found)
		floating_counter++;
	// index patternu posunuji pouze kdyz jsme je namatchovali
	if (match_found == TRUE && floating_counter == FLOATING_COUNTER) {



		findCrossings(&pattern_arr[pat_candidates[active_pattern].left_index],&pattern_arr[pat_candidates[active_pattern].right_index],
				line_arr[index_best_angle_left-1].y,line_arr[index_best_angle_left-1].x,line_arr[index_best_angle_left].y,line_arr[index_best_angle_left].x,
				line_arr[index_best_angle_right].y,line_arr[index_best_angle_right].x,line_arr[index_best_angle_right+1].y,line_arr[index_best_angle_right+1].x,
				&edge_cross_lat, &edge_cross_lon, &pt_cross_lat, &pt_cross_lon);


#ifdef DEBUG
		printf("line [[%lf,%lf],[%lf,%lf],[%lf,%lf],[%lf,%lf]]\n",line_arr[index_best_angle_left-1].y,line_arr[index_best_angle_left-1].x,line_arr[index_best_angle_left].y,line_arr[index_best_angle_left].x,
				line_arr[index_best_angle_right].y,line_arr[index_best_angle_right].x,line_arr[index_best_angle_right+1].y,line_arr[index_best_angle_right+1].x);
		printf("line [[%lf,%lf]]\n",edge_cross_lat, edge_cross_lon);

#endif	//DEBUG



		int res = MoveToLineOnOffset_v2(edge_cross_lat - pt_cross_lat, edge_cross_lon - pt_cross_lon,
				&mm_request_2->gps_pts[pat_candidates[active_pattern].left_index],&mm_request_2->gps_pts[pat_candidates[active_pattern].right_index],
				&pat_candidates[active_pattern].left_candidate[active_candidate],&pat_candidates[active_pattern].right_candidate[active_candidate],
				&pat_candidates[active_pattern].left_line_index[active_candidate],&pat_candidates[active_pattern].right_line_index[active_candidate],
				route_line,route_line_number,index_best_angle_left,p_l,p_r);


		if(!res){ // OK
			matched++;
			active_candidate++;
		}


		//pro pocitani nejlepsiho uhlu
		best_angle_diff = 360;
		//index_best_angle_right = -1;

		// pripravim pro dalsi prohledavani
		match_found = FALSE;

		if(active_candidate >= MAX_PAT_CANDIDATES){
			active_candidate=0;
			//printf("aaa\n");
			break;
		}
	}

	if(floating_counter == FLOATING_COUNTER) {
		floating_counter = 0;
	}

	}
		if(dist_to_pattern > 2*DISTANCE_TOLERANCE && visited == 0)
		{
			visited = 1;
		}

	}


	}
#ifdef DEBUG
	fclose (file_angles);
#endif

	return 1;

}

/*
int FindPatternMatch(MM_REQUEST* mm_request_2, MM_RESPONSE* mm_response_2, POINT_LL* route_line_orig,int route_line_number)
{

	POINT_LL route_line[100000];



	int i,pattern_count,index_pattern_child_left = -1, index_pattern_child_right = -1, matched, last_matched, orientation, old_11_line, old_21_line;

	old_11_line = -1;
	old_21_line = -1;

	for(i=0; i<route_line_number;i++)
	{
		route_line[i].x = route_line_orig[i].x;
		route_line[i].y = route_line_orig[i].y;

	}
	OtocOrientaciLinie(route_line_orig, route_line_number);


	last_matched =-1;
	matched = -1;
	orientation = 0;
	pattern_count = 0;

	while(!FindLRPattern(&index_pattern_child_left, &index_pattern_child_right,mm_request_2,0))
		pattern_count++;

	while(pattern_count>=1){
	while(pattern_count>=1 && (matched || last_matched) ){
		last_matched = matched;
		matched =FindPatternMatchR(mm_request_2,mm_response_2,route_line,route_line_number,orientation%2);
		orientation++;
		pattern_count -= matched;

	}
	if(pattern_count){
		FindLRPattern(&index_pattern_child_left, &index_pattern_child_right,mm_request_2,orientation%2);
		//mm_request_2->gps_pts[index_pattern_child_left].fix_status += 2;
		//mm_request_2->gps_pts[index_pattern_child_right].fix_status += 2;
		pattern_count--;
		last_matched =-1;
		matched = -1;
	}
	}

	for(i=0;i<mm_request_2->gps_pts_size;i++)
	{
		if(mm_request_2->gps_pts[i].fix_status == 11){
			if(old_11_line >= mm_response_2->map_pts[i].match_quality){
				mm_request_2->gps_pts[i].fix_status = 1;
				for(;i<mm_request_2->gps_pts_size;i++)
				{
					mm_response_2->map_pts[i].match_quality = 0;
					if(mm_request_2->gps_pts[i].fix_status == 21){
						mm_request_2->gps_pts[i].fix_status = 1;
						mm_response_2->map_pts[i].match_quality = 0;
					}
				}
			}else
				old_11_line = mm_response_2->map_pts[i].match_quality;

		}
	}

	//if((orientation%2)-1){
	//	OtocOrientaciLinie(route_line, route_line_number);
		//orientation++;
	//}
	return 0;
}
*/
/*double gpsDistance(POINT_LL* line, int start, int end, double lat1, double lon1, double lat2, double lon2)
{
int i;
double total = 0;

for(i = start; i < end; i++) {
total += vzd_by_degree(line[i].x, line[i].y, line[i+1].x, line[i+1].y);
}

total += vzd_by_degree(line[start].x, line[start].y, lon1, lat1);
total -= vzd_by_degree(line[end].x, line[end].y, lon2, lat2);

return total;
}*/

double gpsDistance(MM_POINT_LL* line, int start, int end, double lat1, double lon1, double lat2, double lon2)
{
int i;
double total = 0;

if(start==end){
	return vzd_by_degree(lon1, lat1,lon2, lat2);
}

if(end > start)
{
	for(i = start; i < end-1; i++) {
		total += vzd_by_degree(line[i].x, line[i].y, line[i+1].x, line[i+1].y);
	}

	total += vzd_by_degree(line[start].x, line[start].y, lon1, lat1);
	total += vzd_by_degree(line[end-1].x, line[end-1].y, lon2, lat2);
}
else
{
	for(i = end; i < start-1; i++) {
	total += vzd_by_degree(line[i].x, line[i].y, line[i+1].x, line[i+1].y);
	}

	total += vzd_by_degree(line[start-1].x, line[start-1].y, lon1, lat1);
	total += vzd_by_degree(line[end].x, line[end].y, lon2, lat2);

}


return total;
}

double routeDistance(MM_GPS_PT* points, int index_from, int index_to, int method) {
double distance = 0, time, velocity;
int i;

// spocitam delku routy metodou vzd_by_degree
if(method == 0) {
  for (i = index_from+1; i <= index_to; i++) {
	//if(points[i-1].speed > 5 && points[i].speed > 5)
			distance += vzd_by_degree(points[i-1].longitude, points[i-1].latitude, points[i].longitude, points[i].latitude);
  }
}

// spocitam delku routy metodou s = v * t;
if(method == 1) {
for (i = index_from+1; i <= index_to; i++) {
time = points[i].event_time - points[i-1].event_time;
velocity = (points[i].speed + points[i-1].speed) / 2;
distance += (time * velocity);
}
distance = (distance / 3.6);
}

return distance;
}


int FindPatternMatch(MM_REQUEST* mm_request_2, MM_RESPONSE* mm_response_2, MM_POINT_LL* route_line_orig,int route_line_number)
{

	//POINT_LL route_line[100000];

	MM_PATT_CANDIDATES *pat_candidates;

	double **matrix;

	int i,pattern_count,index_pattern_child_left = -1, index_pattern_child_right = -1,  active_pattern, c , r, p, p2;




	/*for(i=0; i<route_line_number;i++)
{
route_line[i].x = route_line_orig[i].x;
route_line[i].y = route_line_orig[i].y;

}*/
	OtocOrientaciLinie(route_line_orig, route_line_number);



	pattern_count = 0;

	while(!FindLRPattern(&index_pattern_child_left, &index_pattern_child_right,mm_request_2,0))
		pattern_count++;
	if(pattern_count == 0) {
		return 1;
	}
	// uprava 0.3
	pat_candidates = (MM_PATT_CANDIDATES*)calloc(pattern_count, sizeof(MM_PATT_CANDIDATES));
	active_pattern = 0;
	index_pattern_child_left = -1;
	index_pattern_child_right = -1;
	while(!FindLRPattern(&index_pattern_child_left, &index_pattern_child_right,mm_request_2,0))
	{
		pat_candidates[active_pattern].left_index = index_pattern_child_left;
		pat_candidates[active_pattern].right_index = index_pattern_child_right;
		active_pattern++;
	}
	FindPatternMatchR_v2(mm_request_2,route_line_orig,route_line_number,pat_candidates,pattern_count);

#ifdef DEBUG

	/// WRITE CANDIDATES AND MATCHED PATTERNS
	char l1[1024];

	if(file_can_and_matched == NULL)
		file_can_and_matched = fopen("/home/adminuser/workspace/mmnew/Debug/src/candidates_and_matched.csv", "w");
	else
		file_can_and_matched = fopen("/home/adminuser/workspace/mmnew/Debug/src/candidates_and_matched.csv", "a");

	int index,j;
	for(j = 0; j < pattern_count; j++){
		sprintf(l1, "\n[%lf,%lf], [%lf,%lf],\n",
				mm_request_2->gps_pts[pat_candidates[j].left_index].latitude,
				mm_request_2->gps_pts[pat_candidates[j].left_index].longitude,
				mm_request_2->gps_pts[pat_candidates[j].right_index].latitude,
				mm_request_2->gps_pts[pat_candidates[j].right_index].longitude);

		fprintf(file_can_and_matched,l1);
		for(index = 0; index < MAX_PAT_CANDIDATES; index++){
			sprintf(l1, "[%lf,%lf],[%lf,%lf],\n",
					pat_candidates[j].left_candidate[index].y,
					pat_candidates[j].left_candidate[index].x,
					pat_candidates[j].right_candidate[index].y,
					pat_candidates[j].right_candidate[index].x);
			//sprintf(l1, "\n[%i,%i],\n",
			//	pat_candidates[j].left_line_index[index],
			//	pat_candidates[j].right_line_index[index]);
			fprintf(file_can_and_matched,l1);
		}

	}

	fflush(file_can_and_matched);
	fclose (file_can_and_matched);
#endif // DEBUG

#ifdef DEBUGM
	char row[2000];

	if(file_pattern_matched == NULL){
		file_pattern_matched = fopen("/home/adminuser/workspace/mmnew/Debug/src/pattern_matched.csv", "w");
		sprintf(row, "[");
		fputs(row, file_pattern_matched);
	}
	else{
		file_pattern_matched = fopen("/home/adminuser/workspace/mmnew/Debug/src/pattern_matched.csv", "a");}
#endif // DEBUGM

	// allocate matrix
	int size = pattern_count * MAX_PAT_CANDIDATES;
	matrix = (double **)malloc(size * sizeof(double *));
	for(i = 0; i < size; i++)	{
		matrix[i] = (double *)malloc(size * sizeof(double));
	}

	/*for(r = 0; r < size; r++) {
for(c = 0; c < size; c++) {
//diagonal
if(r == c) {
matrix[r][c] = -1;
}
matrix[r][c]
}
}*/
	double dist1, dist2, dist_mc, dist_pat, dist_result;
	for(p = 0; p < pattern_count; p++) {
		for(p2 = p; p2 < pattern_count; p2++) {
			for(r = 0; r < MAX_PAT_CANDIDATES; r++) {
				for (c = 0; c < MAX_PAT_CANDIDATES; c++) {
					if(p == p2) {
						matrix[r+(p*MAX_PAT_CANDIDATES)][c+(p2*MAX_PAT_CANDIDATES)] = -1;
						matrix[c+(p2*MAX_PAT_CANDIDATES)][r+(p*MAX_PAT_CANDIDATES)] = -1;
					} else  {

						if(pat_candidates[p].left_candidate[r].y == -1 || pat_candidates[p].left_candidate[r].x == -1 ||
								pat_candidates[p2].right_candidate[c].y == -1 || pat_candidates[p2].right_candidate[c].x == -1 ||
								pat_candidates[p].right_candidate[r].y == -1 || pat_candidates[p].right_candidate[r].x == -1 ||
								pat_candidates[p2].left_candidate[c].y == -1 || pat_candidates[p2].left_candidate[c].x == -1) {
							matrix[r+(p*MAX_PAT_CANDIDATES)][c+(p2*MAX_PAT_CANDIDATES)] = -1;
							matrix[c+(p2*MAX_PAT_CANDIDATES)][r+(p*MAX_PAT_CANDIDATES)] = -1;
						} else {
							dist1 = 0;
							dist2 = 0;

							if(pat_candidates[p2].right_line_index[c]>10000 || pat_candidates[p2].left_line_index[c]>10000)
								printf("stop\n");
							dist1 = gpsDistance(route_line_orig, pat_candidates[p].left_line_index[r], pat_candidates[p2].right_line_index[c],
									pat_candidates[p].left_candidate[r].y, pat_candidates[p].left_candidate[r].x,
									pat_candidates[p2].right_candidate[c].y, pat_candidates[p2].right_candidate[c].x);

							/*printf("LH: %i, %i, %i, %i\n", p, p2, r, c);
printf("[[%lf, %lf], [%lf, %lf]]\n", pat_candidates[p].left_candidate[r].y, pat_candidates[p].left_candidate[r].x,
pat_candidates[p2].right_candidate[c].y, pat_candidates[p2].right_candidate[c].x);
							 */

							dist2 = gpsDistance(route_line_orig, pat_candidates[p].right_line_index[r], pat_candidates[p2].left_line_index[c],
									pat_candidates[p].right_candidate[r].y, pat_candidates[p].right_candidate[r].x,
									pat_candidates[p2].left_candidate[c].y, pat_candidates[p2].left_candidate[c].x);

							/*printf("[[%lf, %lf], [%lf, %lf]]\n", pat_candidates[p].right_candidate[r].y, pat_candidates[p].right_candidate[r].x,
pat_candidates[p2].left_candidate[c].y, pat_candidates[p2].left_candidate[c].x);
							 */


							dist_mc = (dist1+dist2) / 2;
							//matrix[r+(p*MAX_PAT_CANDIDATES)][c+(p2*MAX_PAT_CANDIDATES)] = (dist1+dist2) / 2;
							//matrix[c+(p2*MAX_PAT_CANDIDATES)][r+(p*MAX_PAT_CANDIDATES)] = (dist1+dist2) / 2;

							dist1 = routeDistance(mm_request_2->gps_pts,pat_candidates[p].left_index, pat_candidates[p2].right_index,0);
							dist2 = routeDistance(mm_request_2->gps_pts,pat_candidates[p].right_index, pat_candidates[p2].left_index,0);
							dist_pat = (dist1+dist2) / 2;

							dist_result = 1-((MAX(fabs(dist_mc), fabs(dist_pat))-MIN(fabs(dist_mc), fabs(dist_pat))) /
									MAX(fabs(dist_mc), fabs(dist_pat)));
							//if(MIN(fabs(dist_mc), fabs(dist_pat) < 100)
							//	dist_result *=


							/*
printf("save on: %i, %i\n", r+(p*MAX_PAT_CANDIDATES), c+(p2*MAX_PAT_CANDIDATES));

printf("dist_mc: %lf, dist_pat: %lf, dist_result: %lf\n\n", dist_mc, dist_pat, dist_result);
							 */
							matrix[r+(p*MAX_PAT_CANDIDATES)][c+(p2*MAX_PAT_CANDIDATES)] = dist_result;
							matrix[c+(p2*MAX_PAT_CANDIDATES)][r+(p*MAX_PAT_CANDIDATES)] = dist_result;

						}


					}

				}
			}
		}
	}



	int best_count = 0;
	int actual_count = 0;
	double actual_sum = 0;
	double best_sum = 0;
	int pat_OK;
	int best_pat_match_index = -1,previous_best_pat_match_index=-1,previous_previous_best_pat_match_index=-1;

	for(r = 0; r < pattern_count*MAX_PAT_CANDIDATES; r++) {

		for (c = 0; c < pattern_count*MAX_PAT_CANDIDATES; c++) {
			if(matrix[r][c] > BEST_PAT_CANDIDATE_LIMIT) {
				actual_sum += matrix[r][c];
				actual_count++;
			}
		}

		/*	pat_OK = TRUE;
if(previous_best_pat_match_index!=-1 && pat_candidates[previous_best_pat_match_index/MAX_PAT_CANDIDATES].right_line_index[previous_best_pat_match_index%MAX_PAT_CANDIDATES]>=pat_candidates[r/MAX_PAT_CANDIDATES].left_line_index[r%MAX_PAT_CANDIDATES])
{
pat_OK=FALSE;
if(previous_previous_best_pat_match_index!=-1)
{

if( matrix[previous_previous_best_pat_match_index][r] > matrix[previous_best_pat_match_index][r])
{

mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].match_quality = 0;
mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].id = 0;
mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].match_quality = 0;

pat_OK = TRUE;

#ifdef DEBUG

printf("mazu:\n[[%lf, %lf], [%lf, %lf]]\n",
mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].latitude,
mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].longitude,
mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].latitude,
mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].longitude);
#endif // DEBUG
}


}

}*/
		//if(pat_OK == TRUE){
		if(actual_count > best_count) {
			best_count = actual_count;
			best_sum = actual_sum;
			best_pat_match_index = r;
		} else if(actual_count == best_count) {
			if(actual_sum > best_sum) {
				best_sum = actual_sum;
				best_pat_match_index = r;
			}
		}
		//}

		actual_count = 0;
		actual_sum = 0;

		if((r+1)%MAX_PAT_CANDIDATES == 0) { //jsem na konci MAX_PAT_CANDIDATateho pruchodu

			//ulozit best pattern match
			if(best_count > 0) {
				pat_OK = TRUE;
				int i_to_pat_pb = previous_best_pat_match_index/MAX_PAT_CANDIDATES;
				int i_to_line_pb = previous_best_pat_match_index%MAX_PAT_CANDIDATES;
				int i_to_pat_r = best_pat_match_index/MAX_PAT_CANDIDATES;
				int i_to_line_r = best_pat_match_index%MAX_PAT_CANDIDATES;
				if(previous_best_pat_match_index!=-1 && pat_candidates[i_to_pat_pb].right_line_index[i_to_line_pb]>=pat_candidates[i_to_pat_r].left_line_index[i_to_line_r])
					//	if(previous_best_pat_match_index!=-1 && pat_candidates[previous_best_pat_match_index/MAX_PAT_CANDIDATES].right_line_index[previous_best_pat_match_index%MAX_PAT_CANDIDATES]>=pat_candidates[best_pat_match_index/MAX_PAT_CANDIDATES].left_line_index[best_pat_match_index%MAX_PAT_CANDIDATES])
				{
					pat_OK=FALSE;
					if(previous_previous_best_pat_match_index!=-1)
					{

						if( matrix[previous_previous_best_pat_match_index][best_pat_match_index] > matrix[previous_best_pat_match_index][best_pat_match_index])
						{

							mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].match_quality = 0;
							mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].id = 0;
							mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].match_quality = 0;

							pat_OK = TRUE;

#ifdef DEBUG

							printf("mazu:\n[[%lf, %lf], [%lf, %lf]]\n",
									mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].latitude,
									mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].longitude,
									mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].latitude,
									mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].longitude);
#endif // DEBUG
						}


					}else{
						dist1 = vzd_by_degree(pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].x,
								pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].y,
								mm_request_2->gps_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_index].longitude,
								mm_request_2->gps_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_index].latitude);
						dist2 = vzd_by_degree(mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].longitude,
								mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].latitude,
								mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].longitude,
								mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].latitude);
						if(dist2>dist1){
							mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].match_quality = 0;
							mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].id = 0;
							mm_response_2->map_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].match_quality = 0;

							pat_OK = TRUE;

#ifdef DEBUG

							printf("mazu:\n[[%lf, %lf], [%lf, %lf]]\n",
									mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].latitude,
									mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].left_index].longitude,
									mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].latitude,
									mm_request_2->gps_pts[pat_candidates[((previous_best_pat_match_index)/MAX_PAT_CANDIDATES)].right_index].longitude);
#endif // DEBUG
						}

					}

				}


				if(pat_OK == TRUE){

					if(previous_best_pat_match_index!=-1)
						previous_previous_best_pat_match_index = previous_best_pat_match_index;
					previous_best_pat_match_index = best_pat_match_index;
					mm_response_2->map_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_index].latitude = pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].y;
					mm_response_2->map_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_index].longitude = pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].x;
					mm_response_2->map_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_index].match_quality = pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_line_index[best_pat_match_index%MAX_PAT_CANDIDATES];
					mm_response_2->map_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_index].id = 999;
					mm_response_2->map_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_index].latitude = pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].y;
					mm_response_2->map_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_index].longitude = pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].x;
					mm_response_2->map_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_index].match_quality = pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_line_index[best_pat_match_index%MAX_PAT_CANDIDATES];
#ifdef DEBUG
					printf("Best Count: %i\n", best_count);
					printf("Pattern:\n[[%lf, %lf], [%lf, %lf]]\n",
							mm_request_2->gps_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_index].latitude,
							mm_request_2->gps_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_index].longitude,
							mm_request_2->gps_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_index].latitude,
							mm_request_2->gps_pts[pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_index].longitude);
					if(best_count > 0) {
						printf("Match for the pattern:\n[[%lf, %lf], [%lf, %lf]]\n\n",
								pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].y,
								pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].x,
								pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].y,
								pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].x);
					} else {
						printf("\n\n");
					}
#endif // DEBUG

#ifdef DEBUGM
					sprintf(row, "[%lf, %lf], [%lf, %lf],\n", pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].y,
							pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].left_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].x,
							pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].y,
							pat_candidates[((r+1)/MAX_PAT_CANDIDATES)-1].right_candidate[best_pat_match_index%MAX_PAT_CANDIDATES].x);
					fputs(row, file_pattern_matched);
#endif // DEBUGM

				}
			}


			best_pat_match_index = -1;
			best_count = 0;
			best_sum = 0;
		}


	}

#ifdef DEBUGM
	//sprintf(row, "]");
	//fputs(row, file_pattern_matched);
	fflush(file_pattern_matched);
	fclose (file_pattern_matched);
#endif //DEBUGM


	// free matrix
	for(i = 0; i < size; i++) {
		free(matrix[i]);
	}
	free(matrix);
	free(pat_candidates);
	// end uprava 0.3


	/*

while(pattern_count>=1){
while(pattern_count>=1 && (matched || last_matched) ){
last_matched = matched;
matched =FindPatternMatchR(mm_request_2,mm_response_2,route_line,route_line_number,orientation%2);
orientation++;
pattern_count -= matched;

}
if(pattern_count){
FindLRPattern(&index_pattern_child_left, &index_pattern_child_right,mm_request_2,orientation%2);
mm_request_2->gps_pts[index_pattern_child_left].fix_status += 2;
mm_request_2->gps_pts[index_pattern_child_right].fix_status += 2;
pattern_count--;
last_matched =-1;
matched = -1;
}
}
	 */


	//if((orientation%2)-1){
	//	OtocOrientaciLinie(route_line, route_line_number);
	//orientation++;
	//}
	return 0;
}



int FillResponse(MM_REQUEST* mm_request_2, MM_RESPONSE* mm_response_2,MM_RESPONSE* mm_response, int start_index)
{
	int z;

	for(z=0;z<mm_request_2->gps_pts_size;z++){
		if(mm_response_2->map_pts[z].match_quality > 0){
		//if(mm_response_2->map_pts[z].match_quality == 10){

			mm_response->map_pts[z+start_index].latitude = mm_response_2->map_pts[z].latitude;
			mm_response->map_pts[z+start_index].longitude = mm_response_2->map_pts[z].longitude;
			mm_response->map_pts[z+start_index].match_quality = mm_response_2->map_pts[z].match_quality;
			if(mm_response_2->map_pts[z].id == 999){
				mm_response->map_pts[z+start_index].id = mm_response_2->map_pts[z].id;
			}

		}
		mm_response_2->map_pts[z].match_quality = 0;
	}

	return 0;
}

int pointScoreDirect(double coef, int headingDiff) {

/*if(headingDiff > 90) {
headingDiff = 180 - headingDiff;
}*/

double score = 1;

if(coef > 1.0)
	coef = (1/coef);

score *= coef;

score *= (180-(double)headingDiff)/180;

return MAX(1,(int)(score*100));
}

int pointScore(double coef, int headingDiff) {

if(headingDiff > 90) {
headingDiff = 180 - headingDiff;
}

double score = 1;

if(coef > 1.0)
	coef = (1/coef);

score *= coef;

score *= (90-(double)headingDiff)/90;

/*int headingDiffR = 180 - headingDiff;

double score = 0;
int topScore = 32400;
double coefR;
coefR = 180 * coef;

score = (coefR * headingDiffR) / topScore;
*/

return MAX(1,(int)(score*100));
}

int FillSegmentPart(int start, int end, MM_REQUEST* mm_request,MM_RESPONSE* mm_response, MM_POINT_LL* route_line,int route_line_number)
{
	double dist_gps,dist_route, coef,draw_at;
	int i,points;

	int  points_drawn=0;

		double  vzd_start, vzd_end, vzd;

		int headingDiff;


	points = end-start-1;
	if(points){
		vzd_end = 0;
	dist_gps = gpsDistance(route_line, mm_response->map_pts[start].match_quality, mm_response->map_pts[end].match_quality,
			mm_response->map_pts[start].latitude,mm_response->map_pts[start].longitude,
			mm_response->map_pts[end].latitude,mm_response->map_pts[end].longitude);
	dist_route = routeDistance(mm_request->gps_pts,start,end,0);



	coef = dist_gps/dist_route;

	/*for(i=start;i<=end;i++){
		if(angleBetweenLineAndPoint(mm_request->gps_pts[i].heading)
	}*/
	draw_at = vzd_by_degree(mm_response->map_pts[start].longitude,mm_response->map_pts[start].latitude,
			route_line[mm_response->map_pts[start].match_quality-1].x,route_line[mm_response->map_pts[start].match_quality-1].y);
	draw_at += vzd_by_degree(mm_request->gps_pts[start].longitude,mm_request->gps_pts[start].latitude,mm_request->gps_pts[start+1].longitude,mm_request->gps_pts[start+1].latitude)*coef;

	for(i = mm_response->map_pts[start].match_quality-1; i<mm_response->map_pts[end].match_quality; i++){
		vzd_start = vzd_end;
		vzd= vzd_by_degree(route_line[i].x, route_line[i].y,route_line[i+1].x, route_line[i+1].y);
		vzd_end += vzd;

		while(vzd_end > draw_at && points_drawn < points){

			points_drawn++;
			mm_response->map_pts[start+points_drawn].longitude=route_line[i].x+((route_line[i+1].x-route_line[i].x)*((double)(draw_at-vzd_start)/vzd));
			mm_response->map_pts[start+points_drawn].latitude=route_line[i].y+((route_line[i+1].y-route_line[i].y)*((double)(draw_at-vzd_start)/vzd));

			headingDiff = angleBetweenLineAndPoint(route_line[i].x, route_line[i].y,route_line[i+1].x, route_line[i+1].y,mm_request->gps_pts[start+points_drawn].heading);
			mm_response->map_pts[start+points_drawn].match_quality= -pointScore(coef, headingDiff);

			draw_at += vzd_by_degree(mm_request->gps_pts[start+points_drawn].longitude,mm_request->gps_pts[start+points_drawn].latitude,mm_request->gps_pts[start+1+points_drawn].longitude,mm_request->gps_pts[start+1+points_drawn].latitude)*coef;
		}
		if(points_drawn == points)
			break;
	}
	}
	//mm_response->map_pts[start].match_quality  +=1000;
	//mm_response->map_pts[end].match_quality += 1000;
	while( points_drawn < points)
	{
		points_drawn++;
		mm_response->map_pts[start+points_drawn].longitude=mm_request->gps_pts[start+points_drawn].longitude;
		mm_response->map_pts[start+points_drawn].latitude=mm_request->gps_pts[start+points_drawn].latitude;
		//mm_response->map_pts[i].match_quality = -300;

				//mm_response->map_pts[i].longitude=mm_request->gps_pts[i].longitude+offset_x;
				//mm_response->map_pts[i].latitude=mm_request->gps_pts[i].latitude+offset_y;
				//mm_response->map_pts[i].match_quality = -300;
	}

	return 0;
}



int FillSegmentPartEnd(int start, int end, MM_REQUEST* mm_request,MM_RESPONSE* mm_response, MM_POINT_LL* route_line,int route_line_number)
{
	double draw_at;
	int i,points,bad_points=0, bad_score=0,score;
	double offset_x,offset_y;
	int  points_drawn=0;
	int headingDiff;
		double  vzd_start, vzd_end, vzd;

		//double degree_coef;

	points = end-start;
   if(points){
		vzd_end = 0;

	draw_at = vzd_by_degree(mm_response->map_pts[start].longitude,mm_response->map_pts[start].latitude,
			route_line[mm_response->map_pts[start].match_quality-1].x,route_line[mm_response->map_pts[start].match_quality-1].y);
	draw_at += vzd_by_degree(mm_request->gps_pts[start].longitude,mm_request->gps_pts[start].latitude,mm_request->gps_pts[start+1].longitude,mm_request->gps_pts[start+1].latitude);



	for(i = mm_response->map_pts[start].match_quality-1; i<route_line_number-1; i++)
	{
		vzd_start = vzd_end;
		vzd= vzd_by_degree(route_line[i].x, route_line[i].y,route_line[i+1].x, route_line[i+1].y);
		vzd_end += vzd;

		while(vzd_end > draw_at && points_drawn < points && bad_score<=5){
			points_drawn++;
			headingDiff = angleBetweenLineAndPoint(route_line[i].x, route_line[i].y,route_line[i+1].x, route_line[i+1].y,mm_request->gps_pts[start+points_drawn].heading);
			score=pointScoreDirect(0.95, headingDiff);
			if(score<70){
				bad_points++;
				bad_score++;
				if(score<40)
						bad_score++;
				}
			if(score>90 && bad_score)
				bad_score--;
			if(bad_score > 5){
					;
			}else{
				mm_response->map_pts[start+points_drawn].longitude=route_line[i].x+((route_line[i+1].x-route_line[i].x)*((double)(draw_at-vzd_start)/vzd));
				mm_response->map_pts[start+points_drawn].latitude=route_line[i].y+((route_line[i+1].y-route_line[i].y)*((double)(draw_at-vzd_start)/vzd));

				mm_response->map_pts[start+points_drawn].match_quality= -score-100;

				draw_at += vzd_by_degree(mm_request->gps_pts[start+points_drawn].longitude,mm_request->gps_pts[start+points_drawn].latitude,mm_request->gps_pts[start+1+points_drawn].longitude,mm_request->gps_pts[start+1+points_drawn].latitude);
			}
		}
		if(points_drawn == points)
			break;
	}
   }
	offset_x = mm_response->map_pts[start+points_drawn-bad_points].longitude-mm_request->gps_pts[start+points_drawn-bad_points].longitude;
	offset_y = mm_response->map_pts[start+points_drawn-bad_points].latitude-mm_request->gps_pts[start+points_drawn-bad_points].latitude;

	/////added
	int routing_end = start+points_drawn-bad_points;
	double offset_ratio = 1/(double)(end-start-points_drawn+bad_points-1);
	double offset_linear = 0;

	for (i=end-1;i>routing_end;i--)///(i=start+points_drawn-bad_points+1;i<end;i++)
	///for (i=start+points_drawn-bad_points+1;i<end;i++)
	{
				///mm_response->map_pts[i].longitude=mm_request->gps_pts[i].longitude+offset_x;
				///mm_response->map_pts[i].latitude=mm_request->gps_pts[i].latitude+offset_y;
				///mm_response->map_pts[i].match_quality = -300;

				/////added
				//mm_response->map_pts[i].longitude=((mm_response->map_pts[routing_end].longitude-mm_request->gps_pts[i].longitude)*offset_linear)+mm_request->gps_pts[i].longitude;
				//mm_response->map_pts[i].latitude=((mm_response->map_pts[routing_end].latitude-mm_request->gps_pts[i].latitude)*offset_linear)+mm_request->gps_pts[i].latitude;
				mm_response->map_pts[i].longitude=mm_request->gps_pts[i].longitude+(offset_x*offset_linear);
				mm_response->map_pts[i].latitude=mm_request->gps_pts[i].latitude+(offset_y*offset_linear);
				mm_response->map_pts[i].match_quality = -300;
				offset_linear += offset_ratio;

	}

	return 0;
}

int FillSegmentPartStart(int start, int end, MM_REQUEST* mm_request,MM_RESPONSE* mm_response, MM_POINT_LL* route_line,int route_line_number)
{
	double draw_at;
	int i,points,bad_points=0, bad_score=0,score;
	double offset_x,offset_y;
	int  points_drawn=0;

	double  vzd_start, vzd_end, vzd,headingDiff ;

	vzd_end = 0;

	points = end-start;
	if(points){

	draw_at = vzd_by_degree(mm_response->map_pts[end].longitude,mm_response->map_pts[end].latitude,
		route_line[mm_response->map_pts[end].match_quality].x,route_line[mm_response->map_pts[end].match_quality].y);
	draw_at += vzd_by_degree(mm_request->gps_pts[end].longitude,mm_request->gps_pts[end].latitude,mm_request->gps_pts[end-1].longitude,mm_request->gps_pts[end-1].latitude);



for(i = mm_response->map_pts[end].match_quality-1; i>=0; i--)
{
	vzd_start = vzd_end;
	vzd= vzd_by_degree(route_line[i].x, route_line[i].y,route_line[i+1].x, route_line[i+1].y);
	vzd_end += vzd;

	while(vzd_end > draw_at && points_drawn < points && bad_score<=5){
		points_drawn++;
		headingDiff = angleBetweenLineAndPoint(route_line[i].x, route_line[i].y,route_line[i+1].x, route_line[i+1].y,mm_request->gps_pts[end-points_drawn].heading);
		score=pointScoreDirect(0.95, headingDiff);

		if(score<70){
			bad_points++;
			bad_score++;
			if(score<40)
					bad_score++;
			}
		if(score>90 && bad_score)
			bad_score--;
		if(bad_score > 5){
			points_drawn--;
		}else{
			mm_response->map_pts[end-points_drawn].longitude=route_line[i+1].x-((route_line[i+1].x-route_line[i].x)*((double)(draw_at-vzd_start)/vzd));
			mm_response->map_pts[end-points_drawn].latitude=route_line[i+1].y-((route_line[i+1].y-route_line[i].y)*((double)(draw_at-vzd_start)/vzd));

			mm_response->map_pts[end-points_drawn].match_quality= -score-100;

			if(points_drawn < points)
				draw_at += vzd_by_degree(mm_request->gps_pts[end-points_drawn].longitude,mm_request->gps_pts[end-points_drawn].latitude,mm_request->gps_pts[end-points_drawn-1].longitude,mm_request->gps_pts[end-points_drawn-1].latitude);
		}
	}
	if(points_drawn == points){
		bad_points = 0;
		break;
	}
}
}

offset_x = mm_response->map_pts[end-points_drawn+bad_points].longitude-mm_request->gps_pts[end-points_drawn+bad_points].longitude;
offset_y = mm_response->map_pts[end-points_drawn+bad_points].latitude-mm_request->gps_pts[end-points_drawn+bad_points].latitude;

/////added
//int routing_start = end-points_drawn+bad_points;
double offset_ratio = 1/(double)(end-points_drawn+bad_points-start);
double offset_linear = 0;

for (i=start;i<end-points_drawn+bad_points;i++)///(i=end-points_drawn+bad_points-1;i>=start;i--)
///for (i=end-points_drawn+bad_points-1;i>=start;i--)
{
			///mm_response->map_pts[i].longitude=mm_request->gps_pts[i].longitude+offset_x;
			///mm_response->map_pts[i].latitude=mm_request->gps_pts[i].latitude+offset_y;
			///mm_response->map_pts[i].match_quality = -300;

			/////added
			//mm_response->map_pts[i].longitude=((mm_response->map_pts[routing_start].longitude-mm_request->gps_pts[i].longitude)*offset_linear)+mm_request->gps_pts[i].longitude;
			//mm_response->map_pts[i].latitude=((mm_response->map_pts[routing_start].latitude-mm_request->gps_pts[i].latitude)*offset_linear)+mm_request->gps_pts[i].latitude;
			mm_response->map_pts[i].longitude=mm_request->gps_pts[i].longitude+(offset_x*offset_linear);
			mm_response->map_pts[i].latitude=mm_request->gps_pts[i].latitude+(offset_y*offset_linear);
			mm_response->map_pts[i].match_quality = -300;
			offset_linear += offset_ratio;
}

return 0;
}


int PointCorrectness(segment* seg,MM_REQUEST* mm_request,MM_RESPONSE* mm_response,int quality_requested)
{
	// define constants
	int limit_value;//10; 	// hodnota znaci limit pro hranici mezi dobrym a spatnym hodnocenim
	const int limit_of_bad_island = 6;//20;//6;	// pokud je velikost island_of_good mensi nez size_of_good_island, pak ostrov dobrych je prislis maly a snizim jim kvalitu
	//const int limit_of_bad_island_300 = 6;

	int i, total_bad, island_of_bad, island_of_good,  i_bad_island_start, i_bad_island_end;
	 int pat_part_start;
	total_bad = island_of_bad = island_of_good = 0;
	i_bad_island_start = i_bad_island_end = -1;
	 pat_part_start = -1;

	 limit_value = quality_requested;
	//pat_start_plus=seg->i_start;
	for(i=seg->i_start;i<seg->i_start+seg->length;i++)
	{

		if(mm_response->map_pts[i].id == 999){
			pat_part_start = i+1;
			island_of_bad = 0;
		}
		if(pat_part_start == -1)
			pat_part_start = i;

		if(!island_of_bad)
			i_bad_island_start = -1;

		if(mm_request->gps_pts[i].speed < MIN_SPEED_FOR_HEADING || (( mm_response->map_pts[i].match_quality > limit_value /*&& mm_response->map_pts[i].match_quality <=100*/) || mm_response->map_pts[i].id ==999 ))
		//if(i == 0 || i == 1 || i == 2 || i == 3 || i == 4 || i == 12 || i == 13 || i == 14 || i == 15 || i == 16 || i == 17 || i == 18)
		{
			if(island_of_bad > 0 && mm_request->gps_pts[i].speed > MIN_SPEED_FOR_HEADING )
				island_of_bad--;
		}
		else
		{
			//if(i_good_island_end > i_bad_island_start && i_good_island_end > i_bad_island_end && island_of_bad > 0) // instead of ZERO use limit_of_bad_island
			island_of_bad++;
			if(i_bad_island_start == -1){
				i_bad_island_start = i;
			}
			if( island_of_bad > limit_of_bad_island)
			{
				// inner cycle for set bad quality
				int inner;

				for(inner = pat_part_start; inner < seg->i_start+seg->length; inner++){
				//for(inner = i_bad_island_start; inner < seg->i_start+seg->length; inner++){
					if(mm_response->map_pts[inner].id ==999 ){
						i=inner;
						pat_part_start = i+1;
						break;
					}else{
						mm_response->map_pts[inner].match_quality = -666;
					}
					//mm_response->map_pts[inner].id = 0;
				}
			// reset
				//i_good_island_start = -1;
				i_bad_island_start = -1;
				island_of_bad = 0;
				//M seg_part_start = -1;
			}

			total_bad++;
			//island_of_good = 0;
			//i_bad_island_end = i;
		}
	}

	int first=1,j, last_err_candidate=-1;;
	for(i=seg->i_start;i<seg->i_start+seg->length;i++)
	{
			if(mm_response->map_pts[i].id ==999 )
			{
				if(i>seg->i_start && i < seg->i_start+seg->length - 1)
				{
					if(mm_response->map_pts[i-1].match_quality == -666 && mm_response->map_pts[i+1].match_quality == -666)
					{
							mm_response->map_pts[i].match_quality = -666;
							mm_response->map_pts[i].id = 0;
					}else{

							if(first)
							{
							 if((mm_response->map_pts[seg->i_start].match_quality ==-666 || mm_response->map_pts[seg->i_start].match_quality >100) &&
							   (mm_response->map_pts[i+1].match_quality == -666 || mm_response->map_pts[i+1].match_quality > 100)){
								 for(j=seg->i_start;j<i+1;j++)
								{
									mm_response->map_pts[j].match_quality = -666;
									mm_response->map_pts[j].id = 0;
								}
							 }
							}
							if(!first)
							{
								if((mm_response->map_pts[i-1].match_quality ==-666 || mm_response->map_pts[i-1].match_quality >100))
								{
									last_err_candidate = i;
								}
								else
								{
									last_err_candidate = -1;
								}
							}
					}

				}
				first = 0;
			}
	}

	if ((last_err_candidate > 0)
			&& (mm_response->map_pts[seg->i_start + seg->length - 1].match_quality == -666 || mm_response->map_pts[seg->i_start + seg->length - 1].match_quality > 100)) {
		for (i=last_err_candidate; i < seg->i_start + seg->length; i++) {
			mm_response->map_pts[i].match_quality = -666;
			mm_response->map_pts[i].id = 0;
		}
	}

#ifdef DEBUG_T8
	printf("Segment corrected, %i bad points from %i\n",total_bad,seg->length);
#endif //DEBUG_T8

	return total_bad;
}

int FillSegment(MM_WS * ws,segment seg, MM_REQUEST* mm_request,MM_RESPONSE* mm_response, MM_POINT_LL* route_line,int route_line_number, int quality_requested)
{
	int i;
	int seg_part_start,seg_part_end, seg_first_start;
	//double offset_x,offset_y;


	seg_part_start= -1;
	seg_part_end = -1;
	seg_first_start = -1;

 //return 1; //xxxxx

	/*for(i=seg->i_start;i<seg->i_start+seg->length;i++)
	{
		if(mm_response->map_pts[i].match_quality >0) // pattern matched
			break;
	}*/


	for(i=seg.i_start;i<seg.i_start+seg.length;i++)
	{
		if(mm_response->map_pts[i].match_quality >0) // pattern matched
		//if(mm_response->map_pts[i].id == 999) // pattern matched

		{
			if(seg_part_start == -1){
				seg_part_start = i;
				if(seg_first_start == -1)
					seg_first_start = i;
			}else
				seg_part_end = i;

			if(seg_part_start>=0 && seg_part_end>0)
			{
				FillSegmentPart( seg_part_start,seg_part_end, mm_request, mm_response, route_line, route_line_number);
				seg_part_start= seg_part_end;
				seg_part_end = -1;
			}

		}
	}
	if(seg_part_start != -1)
	{
		FillSegmentPartStart( seg.i_start,seg_first_start, mm_request, mm_response, route_line, route_line_number);
		/*offset_x = mm_response->map_pts[seg_first_start].longitude-mm_request->gps_pts[seg_first_start].longitude;
		offset_y = mm_response->map_pts[seg_first_start].latitude-mm_request->gps_pts[seg_first_start].latitude;
		for(i=seg->i_start;i<seg_first_start;i++)
		{
			mm_response->map_pts[i].longitude=mm_request->gps_pts[i].longitude+offset_x;
			mm_response->map_pts[i].latitude=mm_request->gps_pts[i].latitude+offset_y;
			mm_response->map_pts[i].match_quality = 1;
		}*/
		FillSegmentPartEnd( seg_part_start,seg.i_start+seg.length, mm_request, mm_response, route_line, route_line_number);
		/*offset_x = mm_response->map_pts[seg_part_start].longitude-mm_request->gps_pts[seg_part_start].longitude;
		offset_y = mm_response->map_pts[seg_part_start].latitude-mm_request->gps_pts[seg_part_start].latitude;
		for(i=seg_part_start+1;i<seg->i_start+seg->length;i++)
		{
			mm_response->map_pts[i].longitude=mm_request->gps_pts[i].longitude+offset_x;
			mm_response->map_pts[i].latitude=mm_request->gps_pts[i].latitude+offset_y;
			mm_response->map_pts[i].match_quality = 1;
		}*/
	}else{
		for(i=seg.i_start;i<seg.i_start+seg.length;i++)
			{
				mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude;
				mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude;
			}
	}


	for(i=seg.i_start;i<seg.i_start+seg.length;i++)
	{
		if(mm_response->map_pts[i].match_quality < 0 )
			mm_response->map_pts[i].match_quality = -mm_response->map_pts[i].match_quality;
	}


	PointCorrectness(&seg, mm_request,mm_response, quality_requested);

	//////////////////////////////////////////////////////////////////////////////////////
/*
	MM_REQUEST unmatched;
	MM_RESPONSE resp_2;
	MM_GPS_PT* gps = NULL;
	MM_MAP_PT* mm = NULL;
	MM_ERRORS um_result;
	int um_counter = 0, first_um = -1, last_um = -1, k, match_quality_limt = 40,j;



	for(i = 0; i < mm_request->gps_pts_size; i++) {
		if(mm_response->map_pts[i].match_quality ==  -666) {
			if(first_um == -1) {
				first_um = i;
			}
			last_um = i;
			um_counter++;

		} else {
			if(um_counter > 50) {
				if(gps != NULL)
				{
					free(gps);
					gps=NULL;
				}
				if(mm != NULL)
				{
					free(mm);
					mm=NULL;
				}
				//unmatched = (MM_REQUEST *)calloc(um_counter, sizeof(MM_REQUEST));
				mm = (MM_MAP_PT*)calloc(1+mm_request->gps_pts_size, sizeof(MM_MAP_PT));
				resp_2.map_pts = mm;

				gps = (MM_GPS_PT*)calloc(1+um_counter, sizeof(MM_GPS_PT));
				unmatched.gps_pts = gps;
				unmatched.gps_pts_size = um_counter;

				for(k = first_um,j=0; k <= last_um; k++,j++) {
					gps[j] = mm_request->gps_pts[k];
				}

				um_result = mm_map_match_old(ws, &unmatched, &resp_2,route_line,&route_line_number);
				findPatterns(&unmatched);
				FindPatternMatch(&unmatched,&resp_2,route_line,route_line_number);
				FillResponse(&unmatched,&resp_2, mm_response,first_um);

				//FillSegment(&seg[no_seg], mm_request, mm_response,route_line,route_line_number);
			}
			um_counter = 0;
			first_um = -1;
		}
	}
	if(gps != NULL)
	{
		free(gps);
		gps=NULL;
	}
	if(mm != NULL)
	{
		free(mm);
		mm=NULL;
	}

*/
	return 0;
}


void clearData(MM_REQUEST* mm_request, MM_RESPONSE* mm_response)
{
	int i;
	for(i=0;i<mm_request->gps_pts_size ;i++){
		mm_request->gps_pts[i].fix_status=1;
		mm_response->map_pts[i].id=0;
		mm_response->map_pts[i].match_quality=0;
	}
}


/**********************************************************************************************************************
 * CONSTANTS
 **********************************************************************************************************************/
#define POINT_TYPE_VALID	0 // point is valid
#define POINT_TYPE_INVALID	1 // point is invalid

// Conditions for point validity
#define MIN_HDOP 9

// Constains for FindPatterns
#define ACC_PTS_COUNT 			3		// count of points before and after curve, where y acceleration must change
#define MIN_ACC_FOR_PATT		0.1		// minimal acceleration value accepted, when looking for pattern candidate
#define MIN_ACC_FOR_PATT_C		1.5
#define PATT_END_LIMIT_ACC	 	0.04//0.05	// limit value for acceleration to be accepted, when finishing pattern
#define GYRO_LIMIT_FOR_PATT 	8//3		// gyro value for points to be accepted as pattern
#define MAX_DIST_PATT_POINT	 	5		// maximal pattern start/end point search distance
#define MIN_GYRO_SUM 			25//20		// sum of gyro values for pattern to be accepted


/**********************************************************************************************************************
 * ENG:
 * CZE: Nalezeni patternu
 **********************************************************************************************************************/
void findPatterns_NV(MM_DATA* data_arr, int seg_start, int seg_length)
{
	int i,j;
	int state = 0;	// set initial state
	int end_count;	// when looking for the end, this is count of already inspected points
	int pat_start;	// index of start point of pattern in original array
	int pat_end = -1;	// index of end point of pattern in original array
	int count_pts;	// true, if counting of points should be done, when searching end point
	//int pat_end;	// index of end point of pattern in original array
	int acc_count;	// count of consecutive points having acceleration higher than specified value
	int first_valid = -1;	// index of first valid point in area inspected
	int negative_y_acc;	// true(1), if y acceleration before curve is negative
	double gyro_sum;	// sum of all gyro for actual pattern
	MM_DATA_STRUCT*	pts = data_arr->pts;

	for (i = seg_start; i < seg_start + seg_length; i++)
	{
		pts[i].fix_status = 0;

		if (pts[i].is_valid == POINT_TYPE_INVALID){	// invalid point
			state = 0;
			first_valid = -1;
			continue;
		}

		switch(state){

		case 0:	// INITIALIZATION
			if(first_valid == -1)
				first_valid = i;

			acc_count = 0;
			state = 1;
			gyro_sum = 0;
			end_count = 0;
			count_pts = 0;

		case 1:	// FIND POINTS HAVING HIGH ACCELERATION

			if (fabs(pts[i].acc_y) > MIN_ACC_FOR_PATT){	// y acceleration accepted

				if(acc_count == 0) {
					if (pts[i].acc_y < 0)
						negative_y_acc = 1;
					else
						negative_y_acc = 0;
				}
				else if ((pts[i].acc_y > (MIN_ACC_FOR_PATT*MIN_ACC_FOR_PATT_C) && negative_y_acc == 1) || (pts[i].acc_y < (-MIN_ACC_FOR_PATT*MIN_ACC_FOR_PATT_C) && negative_y_acc == 0)){
					state = 0;
					break;
				}

				acc_count++;
				gyro_sum += pts[i].gyro_diff;

				if (acc_count == ACC_PTS_COUNT)	// enough points having y acceleration
					state = 2;
				else
					break;
			}
			else{
				state = 0;
				break;
			}

		case 2:	// FIND BACKWARDS START OF THE PATTERN

			for(j = i - ACC_PTS_COUNT; j >= MAX(first_valid, i-ACC_PTS_COUNT+1-MAX_DIST_PATT_POINT); j--){
				if (fabs(pts[j].gyro_diff) < GYRO_LIMIT_FOR_PATT) {	//start point found
					pat_start = j;

					if (pat_start <= pat_end) {	// patterns crossing - forbidden
						state = 0;
					}
					else{
						state = 3;	// from next iteration start looking for the end point
						pat_end = 0;
					}

					break;
				}
				else{
					gyro_sum += pts[j].gyro_diff;
				}
			}
			if (state != 3){	//start point wasn't found
				state = 0;
				break;
			}

		case 3:	// FIND BEST END OF THE PATTERN

			gyro_sum += pts[i].gyro_diff;

			if ((pts[i].acc_y > (MIN_ACC_FOR_PATT*MIN_ACC_FOR_PATT_C) && negative_y_acc == 1) || (pts[i].acc_y < (-MIN_ACC_FOR_PATT*MIN_ACC_FOR_PATT_C) && negative_y_acc == 0)){ // S shape on the line
				if (pat_end){
					state = 4;
				}
				else{
					state = 0;
					break;
				}
			}else if(fabs(pts[i].acc_y) < MIN_ACC_FOR_PATT) {	//start counting
				count_pts = 1;

				if (fabs(pts[i].gyro_diff) < GYRO_LIMIT_FOR_PATT && fabs(gyro_sum) > MIN_GYRO_SUM) {	// some end point found
					pat_end = i;

					if((fabs(pts[i].acc_y) < PATT_END_LIMIT_ACC) || ((pts[i].acc_y >= PATT_END_LIMIT_ACC) && negative_y_acc == 1) ||	((pts[i].acc_y <= -PATT_END_LIMIT_ACC) && negative_y_acc == 0))	{	// best end point found
						state = 4;
					}
				}
			}
			else{
				count_pts = 0;
			}

			if (count_pts){
				end_count++;

				if (end_count == MAX_DIST_PATT_POINT) {	// end of pattern wasn't found
					if (pat_end){
						state = 4;
					}
					else{
						state = 0;
						break;
					}
				}
			}

			if (i == seg_start + seg_length -1 && pat_end) {	// last iteration, not best end, but at least some
				state = 4;
			}

			if (state != 4)
				break;

		case 4:	// WRITE PATTERN

			//	// When angle computation will get fixed, this condition can be removed
			//	if (difangdegSup(pts[pat_start].gps_heading, pts[pat_end].gps_heading) > 90){
			//	state = 0;
			//	break;
			//	}
			pts[pat_start].fix_status = 10;
			pts[pat_end].fix_status = 20;
			state = 0;
			break;
		}
	}
}


#define MIN_GYRO_FOR_PATT_START		2//3
#define MIN_GYRO_FOR_S				10
#define MIN_GYRO_PTS_SKIP_BEWEEN_PATTERNS				3//5
#define MIN_SUM_FOR_END_BIG_PATTERN					80
#define MIN_GYRO_TO_CLOSE_BIG_PATTERN	5

/**********************************************************************************************************************
 * ENG:
 * CZE: Nalezeni patternu
 **********************************************************************************************************************/
void findPatterns_NV_gyro_diff(MM_DATA* data_arr, int seg_start, int seg_length)
{
	int i;
	int gyro_count=0;
	double gyro_sum = 0;	// sum of all gyro for actual pattern
	MM_DATA_STRUCT*	pts = data_arr->pts;
	int pat_start;	// index of start point of pattern in original array
	int negative_gyro;	// true(1), if gyro is negative

	int gyro_end = 0;
	int prev_pat_start = -1, prev_pat_end = -1, prev_gyro_sum;

	//int skipped = MIN_GYRO_PTS_SKIP_BEWEEN_PATTERNS;


	for (i = seg_start+1; i < seg_start + seg_length; i++)
	{
		pts[i].fix_status = 0;

		if (pts[i].is_valid == POINT_TYPE_INVALID){	// invalid point
			gyro_sum = 0;
			gyro_count = 0;
			prev_pat_end = -1;
			continue;
		}

		/*if(skipped < MIN_GYRO_PTS_SKIP_BEWEEN_PATTERNS){
			skipped++;
			continue;
		}*/

		if (fabs(pts[i].gyro_diff) > MIN_GYRO_FOR_PATT_START)
		{

			if(gyro_count == 0){
				if (pts[i].gyro_diff < 0)
					negative_gyro = 1;
				else
					negative_gyro = 0;

				//if(s_curve != 1 || negative_gyro == s_negative_gyro)
				//{

				if(i>1 && /*prev_pat_end < i-2 &&*/ (fabs(pts[i-1].gyro_diff) < MIN_GYRO_FOR_PATT_START))
				{
					pat_start = i-2;
				}else{
					if(i>1 /*&& prev_pat_end < i-1*/)
						pat_start = i-1;
					else
						pat_start = i;
				}

				gyro_sum = pts[i].gyro_diff;
				gyro_count++;
				//}

			}else{
				if ((pts[i].gyro_diff > MIN_GYRO_FOR_PATT_START && negative_gyro == 0) || (pts[i].gyro_diff < -MIN_GYRO_FOR_PATT_START && negative_gyro == 1))
				{ // still cornering in direction
					gyro_sum += pts[i].gyro_diff;
					gyro_count++;
					if(gyro_sum > MIN_SUM_FOR_END_BIG_PATTERN)
						if(pts[i].gyro_diff < MIN_GYRO_TO_CLOSE_BIG_PATTERN)
							gyro_end = 1;
					/*				}else if((pts[i].gyro_diff < -MIN_GYRO_FOR_S && negative_gyro == 0) || (pts[i].gyro_diff > MIN_GYRO_FOR_S && negative_gyro == 1))
				{ // S curve, no pattern
					gyro_count = 0;
					i--;
					 */
				}else{ // end off pattern
					gyro_end = 1;
					if((pts[i].gyro_diff < -MIN_GYRO_FOR_PATT_START && negative_gyro == 0) || (pts[i].gyro_diff > MIN_GYRO_FOR_PATT_START && negative_gyro == 1))
					{
						i--;
					}
				}
			}
		}else{
			if(gyro_count)
				gyro_end = 1;

		}

		if(gyro_end){
			gyro_end = 0;
			if(fabs(gyro_sum) > MIN_GYRO_SUM){
				if(((pat_start - prev_pat_end) > MIN_GYRO_PTS_SKIP_BEWEEN_PATTERNS)){
					pts[pat_start].fix_status = 10;
					pts[i].fix_status = 20;
					prev_pat_start = pat_start;
					prev_pat_end = i;
					prev_gyro_sum = gyro_sum;
					//skipped = 0;
				}else{
					if(fabs(gyro_sum) > fabs(prev_gyro_sum)){
						//pts[pat_start].fix_status = 10;
						//pts[i].fix_status = 20;
						if(prev_pat_end > -1){
							pts[prev_pat_start].fix_status = 0;
							pts[prev_pat_end].fix_status = 0;
						}
						pts[pat_start].fix_status = 10;
						pts[i].fix_status = 20;
						prev_pat_start = pat_start;
						prev_pat_end = i;
						prev_gyro_sum = gyro_sum;
					}else{
						; // no action necessary
					}
				}
			}
			gyro_count = 0;
		}
	}

}


/**********************************************************************************************************************
 * ENG: New version of mm_map_match
 * CZE: Nova verze mm_map_match
 **********************************************************************************************************************/
MM_ERRORS mm_map_match_NV(MM_WS * ws, MM_DATA* data_arr, int seg_start, int seg_length, MM_RESPONSE* mm_response,MM_POINT_LL* route_line,int* route_line_number)
{
	MM_ERRORS result;
	MM_REQUEST_W mm_request_recursive;
	MM_RESPONSE_W mm_response_recursive;
	int i, gpscount, pts_skipped = 0, last_valid=seg_start, low_diff_count=0, hed_dif=0, first=-1,last=-1;
	//int hed_last;
	MM_GPS_PT_W* gps_pts;
	MM_MAP_PT_W* mm_pts;
	double vzd_gps =0;
	int head_dif, seg_quality=0;
	int only_one_route_line;

	gps_pts = (MM_GPS_PT_W*)calloc(1+data_arr->arr_size, sizeof(MM_GPS_PT_W));
	mm_pts = (MM_MAP_PT_W*)calloc(1+data_arr->arr_size, sizeof(MM_MAP_PT_W));

	mm_request_recursive.gps_pts = gps_pts;
	mm_response_recursive.map_pts = mm_pts;

	mm_response->map_pts[0].latitude = data_arr->pts[seg_start].latitude;
	mm_response->map_pts[0].longitude = data_arr->pts[seg_start].longitude;

	// add first point
	/*
	gps_pts[0].latitude = data_arr->pts[seg_start].latitude;
	gps_pts[0].longitude = data_arr->pts[seg_start].longitude;
	gps_pts[0].heading = data_arr->pts[seg_start].gps_heading;	//TODO - gyro heading instead?
	gps_pts[0].speed = data_arr->pts[seg_start].gps_speed;
	gps_pts[0].index = 0;

	gpscount = 1;
	*/
	gpscount = 0;
	//hed_last = gps_pts[0].heading;


//	if (data_arr->arr_size == 1)
//	{
//		mm_response->map_pts[0].latitude = data_arr->pts[0].latitude;
//		mm_response->map_pts[0].longitude = data_arr->pts[0].longitude;
//	}

	for (i = seg_start; i < seg_start+seg_length;i++)
		{

			if (data_arr->pts[i].is_valid == POINT_TYPE_INVALID){
				if(i == (seg_start+seg_length)-1){
					for (; i > seg_start;i--){
						if (data_arr->pts[i].is_valid == POINT_TYPE_VALID){
							if(i>last){
								first = -2;
								break;
							}
						}
						if(i<=seg_start+1){
							first = 1; // no end point valid
							i = seg_start+seg_length; //end of for
							break;
						}
					}

				}
				if(first!=-2){
					//pts_skipped++;
					continue;
				}
			}

		vzd_gps += vzd_by_degree(data_arr->pts[last_valid].longitude,data_arr->pts[last_valid].latitude,data_arr->pts[i].longitude,data_arr->pts[i].latitude);
		last_valid = i;

		// OLD
		/*
		if(data_arr->pts[i].speed>10){
			hed_dif = abs(hed_last - data_arr->pts[i].gps_heading);	//TODO - gyro heading instead?
			if(hed_dif >180)
				hed_dif -= 360;
		}
		*/


		// NEW

		if(abs(data_arr->pts[i].gyro_diff) < 3.0)
		{
			low_diff_count++;
			if(low_diff_count == 3){ hed_dif = 0; low_diff_count = 0; }
		}
		else if(data_arr->pts[i].is_valid == POINT_TYPE_VALID)
		{
			hed_dif += data_arr->pts[i].gyro_diff;
			low_diff_count = 0;
		}



		if (first < 0  || vzd_gps > MAX_GPS_VZD || abs(hed_dif) > MAX_HED_DIF || pts_skipped > MAX_GPS_PTS_SKIP)
		{
			gps_pts[gpscount].latitude = data_arr->pts[i].latitude;
			gps_pts[gpscount].longitude = data_arr->pts[i].longitude;
			gps_pts[gpscount].heading = data_arr->pts[i].gps_heading;
			gps_pts[gpscount].speed = data_arr->pts[i].gps_speed;
			gps_pts[gpscount].index = i;
			gpscount++;
			if(first==-2)
				break;
			pts_skipped = 0;
			vzd_gps = 0;
			//hed_last = data_arr->pts[i].gps_heading;
			hed_dif = 0;
			low_diff_count = 0;
			last = i;
			first = 1;
		}
		else
		{
			pts_skipped++;
		}
	}

	if(gpscount){
		mm_request_recursive.gps_pts_size = gpscount;
		result = mm_map_match_recursive(ws, IMM_OSM, &mm_request_recursive, &mm_response_recursive, route_line,route_line_number,&only_one_route_line);
	}else
		result = MM_NO_VALID_POINT;

#ifdef DEBUGM

	int z;
	char vypis_gps[20000],tempchar[2048];
	sprintf(vypis_gps,"[");



	char vypis_mm[20000],tempchar_mm[2048];
	sprintf(vypis_mm,"[");

	if(file_original_filtered == NULL)
		file_original_filtered = fopen("/home/adminuser/workspace/mmnew/Debug/src/original_filtered.csv", "w");
	else
		file_original_filtered = fopen("/home/adminuser/workspace/mmnew/Debug/src/original_filtered.csv", "a");

	if(file_mm_filtered == NULL)
		file_mm_filtered = fopen("/home/adminuser/workspace/mmnew/Debug/src/mm_filtered.csv", "w");
	else
		file_mm_filtered = fopen("/home/adminuser/workspace/mmnew/Debug/src/mm_filtered.csv", "a");


	//char vypis_r[50000];
	//sprintf(vypis_r,"[");

	for (z=0; z < mm_request_recursive.gps_pts_size;z++)
	{
		sprintf(tempchar,"[%f,%f],", gps_pts[z].latitude, gps_pts[z].longitude);
		strcat(vypis_gps,tempchar);

		if (z % 100 == 0)
		{
			fputs(vypis_gps,file_original_filtered);
			sprintf(vypis_gps, "");
		}
	}

	fputs(vypis_gps, file_original_filtered);
	fputs("]\n\n", file_original_filtered);
	fclose(file_original_filtered);

	for (z=0; z < mm_request_recursive.gps_pts_size;z++)
	{
		sprintf(tempchar_mm,"[%f,%f],", mm_response_recursive.map_pts[gps_pts[z].index].latitude, mm_response_recursive.map_pts[gps_pts[z].index].longitude);
		strcat(vypis_mm,tempchar_mm);

		if (z % 100 == 0)
		{
			fputs(vypis_mm, file_mm_filtered);
			sprintf(vypis_mm, "");
		}
	}

	fputs(vypis_mm, file_mm_filtered);
	fputs("]\n\n", file_mm_filtered);
	fclose(file_mm_filtered);


//	for (z=0; z < *route_line_number;z++)
//	{
//
//		sprintf(tempchar,"[%f,%f],", route_line[z].y, route_line[z].x);
//		//sprintf(tempchar,"%f,%f\n", route_line[z].y, route_line[z].x);
//		strcat(vypis_r,tempchar);
//
//	}

#endif //DEBUGM



	if(result != MM_OK)
	{
		for (i = seg_start; i < seg_start+seg_length; i++)
		{
			mm_response->map_pts[i].latitude = data_arr->pts[i].latitude;
			mm_response->map_pts[i].longitude = data_arr->pts[i].longitude;
		}
		seg_quality = seg_length;
	}else{
		for (i = seg_start; i < seg_start+seg_length; i++)
		{
			if(mm_response_recursive.map_pts[i].geocoded){
				mm_response->map_pts[i].latitude = mm_response_recursive.map_pts[i].latitude;
				mm_response->map_pts[i].longitude = mm_response_recursive.map_pts[i].longitude;
				head_dif = abs(data_arr->pts[i].gps_heading - mm_response_recursive.map_pts[i].heading);
				head_dif = fmod(head_dif,180);
				if(head_dif>90)
					head_dif = 180 - head_dif;
				if(head_dif > 30){
					seg_quality += 1;
				}
			}else{
				mm_response->map_pts[i].latitude = data_arr->pts[i].latitude;
				mm_response->map_pts[i].longitude = data_arr->pts[i].longitude;
				seg_quality += 1;
			}
		}
	}

	if((double)seg_quality/(seg_length) > 0.25){
		mm_response->quality += 1;	// count of bad segments is stored here now
	}

	free(gps_pts);
	free(mm_pts);

	return MM_OK;	//TODO - po opusteni funkce prepocitat mm_response->quality na procenta : 1-((double)bad_segments/segments_count)) * 100
}


#define MIN_VALID_CAN_SPEED 9
#define MIN_VALID_RPM 1100
/**********************************************************************************************************************
 * ENG: Set valid and invalid points
 * CZE: Oznaci zda je bod validni nebo ne podle
 **********************************************************************************************************************/
/*
void SetValidity(MM_DATA* data_arr)
{
	// TMP
	BOOL isNewData = FALSE, hasCANspeed = FALSE;
	int x;
	for(x=0; x < data_arr->arr_size; x++)
	{
		if(data_arr->pts[x].HDOP > 0)
		{
			isNewData = TRUE;
			break;
		}
	}
	// TMP END

	int i, isRpmAndSpeedOK = 0;
	BOOL condition1 = FALSE;

	if(isNewData){
		// FORWARDS
		for(i=0; i < data_arr->arr_size; i++)
		{
			if( isRpmAndSpeedOK == 0 &&
					(data_arr->pts[i].can_speed > MIN_VALID_CAN_SPEED &&
					data_arr->pts[i].RPM > MIN_VALID_RPM))
			{
				isRpmAndSpeedOK = 1;
			}

			// CONDITION #1 - HDOP
			condition1 = (data_arr->pts[i].HDOP > MIN_HDOP) ? FALSE : TRUE;

			if(condition1 && isRpmAndSpeedOK)
			{
				data_arr->pts[i].is_valid = POINT_TYPE_VALID;
			}
			else
			{
				data_arr->pts[i].is_valid = POINT_TYPE_INVALID;
			}
		}

		// BACKWARDS
		isRpmAndSpeedOK = 0;
		for(i=data_arr->arr_size-1; i>=0; i--)
		{
			if( isRpmAndSpeedOK == 0 && data_arr->pts[i].can_speed > MIN_VALID_CAN_SPEED && data_arr->pts[i].RPM > MIN_VALID_RPM){ isRpmAndSpeedOK = 1;}

			if(isRpmAndSpeedOK == 0)
			{
				data_arr->pts[i].is_valid = POINT_TYPE_INVALID;
			}
		}
	}
	else
	{
		// TMP
		for(i=0; i < data_arr->arr_size; i++)
		{
			if(data_arr->pts[i].can_speed > 0)
			{
				hasCANspeed = TRUE;
				break;
			}
		}

		if(hasCANspeed)
		{
			for(i=0; i < data_arr->arr_size; i++)
			{
				if(data_arr->pts[i].can_speed > MAX_SPEED_FOR_DUTCH){ data_arr->pts[i].is_valid = POINT_TYPE_VALID; }
				else {data_arr->pts[i].is_valid = POINT_TYPE_INVALID;}
				data_arr->pts[i].gps_speed = data_arr->pts[i].can_speed;
			}
		}
		else
		{
			for(i=0; i < data_arr->arr_size; i++)
			{
				if(data_arr->pts[i].gps_speed > MAX_SPEED_FOR_DUTCH){ data_arr->pts[i].is_valid = POINT_TYPE_VALID; }
				else {data_arr->pts[i].is_valid = POINT_TYPE_INVALID;}
			}
		}
		// TMP - END
	}
}
*/

void SetValidity2(MM_DATA* data_arr)
{
	BOOL condition1 = FALSE, condition2 = FALSE, condition3 = FALSE, condition4 = FALSE, isNewData = FALSE, hasCANspeed = FALSE, hasFixStatus = FALSE/*, hasAccelerometer = FALSE*/;
	int isRpmAndSpeedOK = 0;
	int i;

	for(i=0; i < data_arr->arr_size; i++)
	{
		// set default
		data_arr->pts[i].is_valid = POINT_TYPE_VALID;

		// test if getting important values like HDOP, CAN_SPEED, ACCELEROMETER and etc.
		if(data_arr->pts[i].HDOP > 0){ isNewData = TRUE; }
		if(data_arr->pts[i].can_speed > 0){ hasCANspeed = TRUE; }
		if(data_arr->pts[i].fix_status > 0){ hasFixStatus = TRUE; }
		//if(data_arr->pts[i].acc_x > 0){ hasAccelerometer = TRUE; }
	}

	// FORWARDS
	for(i=0; i < data_arr->arr_size; i++)
	{
		if( isRpmAndSpeedOK == 0 && (data_arr->pts[i].can_speed > MIN_VALID_CAN_SPEED && data_arr->pts[i].RPM > MIN_VALID_RPM))
		{
			isRpmAndSpeedOK = 1;
		}

		// CONDITION #1 - HDOP
		if(isNewData && data_arr->pts[i].HDOP > MIN_HDOP){ condition1 = TRUE; }
		else{ condition1 = FALSE;}

		// CONDITION #2 - FIX STATUS
		if( hasFixStatus && data_arr->pts[i].fix_status < 1){ condition2 = TRUE; }
		else{ condition2 = FALSE; }

		// CONDITION #3 - CAN SPEED - OLD DATA!!!
		if(!isNewData && hasCANspeed && data_arr->pts[i].can_speed <= MAX_SPEED_FOR_DUTCH){ condition3 = TRUE;}
		else{ condition3 = FALSE; }

		// CONDITION #4 - GPS SPEED - OLD DATA!!!
		if(!isNewData && !hasCANspeed && data_arr->pts[i].gps_speed <= MAX_SPEED_FOR_DUTCH){ condition4 = TRUE; data_arr->pts[i].gps_speed = data_arr->pts[i].can_speed;}
		else {condition4 = FALSE; }

		if(condition1 || condition2 || condition3 || condition4 || !isRpmAndSpeedOK)
		{
			data_arr->pts[i].is_valid = POINT_TYPE_INVALID;
		}
	}

	// BACKWARDS
	isRpmAndSpeedOK = 0;
	for(i=data_arr->arr_size-1; i>=0; i--)
	{
		if( isRpmAndSpeedOK == 0 && data_arr->pts[i].can_speed > MIN_VALID_CAN_SPEED && data_arr->pts[i].RPM > MIN_VALID_RPM){ isRpmAndSpeedOK = 1;}

		if(isRpmAndSpeedOK == 0)
		{
			data_arr->pts[i].is_valid = POINT_TYPE_INVALID;
		}
	}

}
/**********************************************************************************************************************
 * ENG: Calculate new heading using GYRO difference
 * CZE: Vypocita heading pomoci hodnot z GYROSKOPU a nastavi novy heading
 **********************************************************************************************************************/
void CalculateGyroHeading(MM_DATA* data_arr)
{
	int i, straight_num = 0;
	double sum = 0, new_angle = 0, ALPHA = -1;
	MM_DATA_STRUCT*	pts = data_arr->pts;

	pts[0].gyro_heading = (double)pts[0].gps_heading;

	for(i=1; i < data_arr->arr_size; i++)
	{
		pts[i].gyro_heading = (double)pts[i].gps_heading;

		// check if point is valid
		if(pts[i].is_valid == POINT_TYPE_INVALID || pts[i-1].is_valid == POINT_TYPE_INVALID){ continue; }
		if(pts[i].gyro_diff == 0){ continue; } // gyro difference MISSING

		if(abs(pts[i].gyro_diff) < 3)
		{
			straight_num++;
			if(straight_num > 4)
			{
				ALPHA = -1;
				continue;
			}
		}else{
			straight_num = 0;}

		sum = pts[i].gps_heading - pts[i-1].gyro_heading + pts[i].gyro_diff;
		if( fabs(sum) > 2.0 )
		{
			if(ALPHA == -1){ALPHA = pts[i-1].gps_heading;}
			new_angle = ALPHA - pts[i].gyro_diff;

			if(new_angle < 0) new_angle+=360;
			else if(new_angle > 360) new_angle-=360;

			ALPHA = new_angle;
			pts[i].gyro_heading = new_angle;
		}
		else
		{
			ALPHA = -1;
		}
	}
}


void CreateGyroDif(MM_DATA* data_arr)
{
	int i;

	MM_DATA_STRUCT*	pts = data_arr->pts;

	pts[0].gyro_diff = 0;

	for(i=1; i < data_arr->arr_size; i++)
	{
		pts[i].gyro_diff = pts[i].gps_heading-pts[i-1].gps_heading;
		if(pts[i].gyro_diff < -180)
			pts[i].gyro_diff+=360;
		else if(pts[i].gyro_diff > 180)
			pts[i].gyro_diff-=360;
	}

}

/**********************************************************************************************************************
 * ENG: Set segment number for points - CORE
 * CZE: Nastavi kazdemu bodu cislo segmentu - CORE
 **********************************************************************************************************************/
void SetSegmentCore(MM_DATA_STRUCT*	pts, int first_valid, int segment_size, int* actual_segment)
{
	int i;
	for(i = first_valid; i < first_valid + segment_size; i++)
	{
		pts[i].segment_no = *actual_segment;
	}
	(*actual_segment)++;
}


/**********************************************************************************************************************
 * ENG: Set segment number for points
 * CZE: Nastavi kazdemu bodu cislo segmentu
 **********************************************************************************************************************/
void SetSegment(MM_WS * ws, MM_DATA* data_arr)
{
	MM_DATA_STRUCT*	pts = data_arr->pts;
	int i, first = -1, last = -1, segment_size = -1, size = -1, actual_segment = 0;
	for(i=0; i < data_arr->arr_size; i++)
	{
		if(first == -1){ first = i;}
		last = i;

		// oroginal
		//if(i == data_arr->arr_size-1 || (abs(pts[i+1].event_time - pts[i].event_time)) > FD_MAX_SPACE_BETWEEN_POINTS)
		//new
		if( i == data_arr->arr_size-1 ||
			(abs(pts[i+1].event_time - pts[i].event_time)) > FD_MAX_SPACE_BETWEEN_POINTS ||
			vzd_by_degree(pts[i+1].longitude, pts[i+1].latitude, pts[i].longitude, pts[i].latitude) > MAX_DIFF_FOR_NEW_SEGMENT)
		{
			do
			{
				size = last - first + 1;
				segment_size = find_reasonable_count_2(ws, IMM_OSM, &pts[first], size);
				// set segment NO. for points
				SetSegmentCore(pts, first, segment_size, &actual_segment);

				first = first + segment_size;

			}while(segment_size != size);
			first = -1;
			last = -1;
		}
	}
}


/**********************************************************************************************************************
 * ENG: Copy data from MM_REQUEST structure to MM_DATA structure
 * CZE: Nakopirovani data z mm_request do noveho pole [typu MM_DATA]
 **********************************************************************************************************************/
int Convert_MM_REQUEST2MM_DATA(MM_REQUEST* mm_request, MM_DATA* arr)
{
	int i;
	for(i=0; i < mm_request->gps_pts_size; i++)
	{
		arr->pts[i].RPM 			= mm_request->gps_pts[i].RPM;
		arr->pts[i].device_id 		= mm_request->gps_pts[i].device_id;
		arr->pts[i].dist_covered 	= mm_request->gps_pts[i].dist_covered;
		arr->pts[i].event_time 		= mm_request->gps_pts[i].event_time;
		arr->pts[i].fix_status 		= mm_request->gps_pts[i].fix_status;
		arr->pts[i].gps_heading		= mm_request->gps_pts[i].heading;
		arr->pts[i].jny_end 		= mm_request->gps_pts[i].jny_end;
		arr->pts[i].jny_start 		= mm_request->gps_pts[i].jny_start;
		arr->pts[i].latitude 		= mm_request->gps_pts[i].latitude;
		arr->pts[i].longitude 		= mm_request->gps_pts[i].longitude;
		arr->pts[i].sats_count 		= mm_request->gps_pts[i].sats_count;
		arr->pts[i].gps_speed 			= mm_request->gps_pts[i].speed;
		arr->pts[i].gyro_diff		= mm_request->gps_pts[i].gyro_diff;
		arr->pts[i].HDOP			= mm_request->gps_pts[i].HDOP;
		arr->pts[i].acc_x			= mm_request->gps_pts[i].acc_x;
		arr->pts[i].acc_y			= mm_request->gps_pts[i].acc_y;
		arr->pts[i].can_speed		= mm_request->gps_pts[i].can_speed;
	}

	arr->arr_size = mm_request->gps_pts_size;
	return 1;
}


/**********************************************************************************************************************
 * ENG: Copy values from gyro_heading to gps_heading
 * CZE: Prevede gyro_heading do gps_heading
 **********************************************************************************************************************/
void CopyGyro2Heading(MM_DATA* data_arr)
{
	int i;
	for(i=1; i < data_arr->arr_size; i++)
	{
		data_arr->pts[i].gps_heading = data_arr->pts[i].gyro_heading;
	}
}


/**********************************************************************************************************************
 * ENG: Get segment start and segment length
 * CZE: Vrati index a delku segmentu
 * return 0 = OK, 1 = Segment not exist
 **********************************************************************************************************************/
int GetSegment(MM_DATA* data_arr, int segment_no, int *segment_start, int *segment_length)
{
	int i;
	(*segment_start) = 0;
	(*segment_length) = 0;
	for(i=0; i < data_arr->arr_size; i++)
	{
		if(data_arr->pts[i].segment_no == segment_no /*&& data_arr->pts[i].is_valid == POINT_TYPE_VALID*/)
		{
			if((*segment_length) == 0)
			{
				(*segment_start) = i;
			}
			(*segment_length)++;
		}
	}

	if((*segment_length) > 0){	return 0; }
	else { return 1; }
}

int CreateSegment(MM_WS * ws, MM_DATA* data_arr, int *segment_start, int *segment_length)
{

	MM_DATA_STRUCT*	pts = data_arr->pts;
	int i, segment_size = -1, size = -1;
	for(i=*segment_start; i < data_arr->arr_size; i++)
	{

		if( i == data_arr->arr_size-1 ||
			(abs(pts[i+1].event_time - pts[i].event_time)) > FD_MAX_SPACE_BETWEEN_POINTS ||
			vzd_by_degree(pts[i+1].longitude, pts[i+1].latitude, pts[i].longitude, pts[i].latitude) > MAX_DIFF_FOR_NEW_SEGMENT)
		{
				size = i - *segment_start + 1;
				segment_size = find_reasonable_count_2(ws, IMM_OSM, &pts[*segment_start], size);
				*segment_length = segment_size;
				if(segment_size == size)
					return 1; // no splitting because of large data
				else
					return 2;
		}
	}
	return 0;
}

/**********************************************************************************************************************
 * ENG: Create file with debug information
 * CZE: Kontrolni vypis obsahu poli do souboru
 **********************************************************************************************************************/
int ControlPrint(MM_REQUEST* mm_request, MM_DATA* data_arr)
{
	FILE* fw = NULL;
	fw = fopen("/home/adminuser/workspace/mmnew/Debug/src/control.csv", "w");
	char row[2000];

	sprintf(row, "*********************************\n");
	fputs(row, fw);

	sprintf(row, "Is valid =  %i, Is invalid = %i\n", POINT_TYPE_VALID, POINT_TYPE_INVALID);
	fputs(row, fw);

	sprintf(row, "*********************************\n");
	fputs(row, fw);

	int a, no_of_valid = 0, no_of_invalid = 0, no_of_valid_seg = 0, no_of_invalid_seg = 0;
	for(a=0; a < data_arr->arr_size; a++)
	{
		if(data_arr->pts[a].is_valid == 0)
		{ no_of_valid++;}

		if(data_arr->pts[a].is_valid == 1)
		{no_of_invalid++;}
	}

	for(a=0; a < data_arr->arr_size; a++)
	{
		sprintf(row, "[%i] [%i] [%f %f] gps_heading:[%i], gyro_heading:[%lf]\n",a, data_arr->pts[a].is_valid, data_arr->pts[a].latitude, data_arr->pts[a].longitude, data_arr->pts[a].gps_heading, data_arr->pts[a].gyro_heading);
		fputs(row, fw);
	}


	int actual_segment_no = 0, actual_segment_start = -1, actual_segment_length = -1, result;
	do
	{
		result = GetSegment(data_arr, actual_segment_no, &actual_segment_start, &actual_segment_length);
		no_of_valid_seg = 0, no_of_invalid_seg = 0;

		for(a=actual_segment_start; a < actual_segment_start + actual_segment_length; a++)
		{
			if(data_arr->pts[a].is_valid == 0)
			{ no_of_valid_seg++;}

			if(data_arr->pts[a].is_valid == 1)
			{no_of_invalid_seg++;}
		}

		if(result == 1)
		{
			break;
		}
		sprintf(row, "segment:[%i], segment start:[%i], segment length:[%i], valid pts:[%i], invalid pts:[%i]\n", actual_segment_no, actual_segment_start, actual_segment_length, no_of_valid_seg, no_of_invalid_seg);
		fputs(row, fw);
		sprintf(row, "segment:[%i], [%lf,%lf],[%lf,%lf]\n",actual_segment_no,
				data_arr->pts[actual_segment_start].latitude,
				data_arr->pts[actual_segment_start].longitude,
				data_arr->pts[actual_segment_start+actual_segment_length-1].latitude,
				data_arr->pts[actual_segment_start+actual_segment_length-1].longitude);
		fputs(row, fw);

		actual_segment_no++;

	}while(TRUE);

	//sprintf(row, "mm_request:[%i], data_arr:[%i]\n", mm_request->gps_pts_size, data_arr->arr_size);
	//fputs(row, fw);

	sprintf(row, "Overall valid pts:[%i], invalid pts:[%i]\n", no_of_valid, no_of_invalid);
	fputs(row, fw);

	sprintf(row, "*END*\n");
	fputs(row, fw);

	fflush(fw);
	fclose (fw);
	return 1;
}


// TMP
// Stara verze metody s novou datovou strukturou
/*
* urcuje jestli jsou dva uhly stejne v ramci tolerance
*/
int HeadingDifference_OV(MM_DATA_STRUCT pt1, MM_DATA_STRUCT pt2) {
	int result ;

	if(pt1.gps_speed < MIN_SPEED_FOR_HEADING || pt2.gps_speed < MIN_SPEED_FOR_HEADING)
		return 0;

	result = difangdeg(pt1.gps_heading, pt2.gps_heading);

	if (result > HEADING_TOLERANCE) {
		return result;
	}
	return 0;

}
int IsPattern2_OV(MM_DATA_STRUCT* points, int no_controlled_points, int *index1, int *index2) {
	int i, j;
	int Max_Heading = 0;
	int Actual_Heading = 0;
	//for (i = 0; i <= no_controlled_points - 1; i++) {// uprava JH
	i=0;
	for (j = i + 1; j <= no_controlled_points - 1; j++) {
		Actual_Heading = HeadingDifference_OV(points[i], points[j]);
		if (Actual_Heading > Max_Heading && points[i].is_valid == POINT_TYPE_VALID && points[j].is_valid == POINT_TYPE_VALID) {
			Max_Heading = Actual_Heading;
			*index1 = i;
			*index2 = j;
			if(Max_Heading>MAX_ANGLE_FOR_PAT){
				break;
			}
		}
		if(vzd_by_degree(points[i].longitude,points[i].latitude,points[j].longitude,points[j].latitude)>MAX_DIST_FOR_PAT)
			break;
	}
	return Max_Heading;
	//}
	//if (Max_Heading) {
	//return TRUE;
	//} else {
	//return FALSE;
	//}
}

void findPatterns_OV(MM_DATA* data_arr, int seg_start, int seg_length)
{
	int k;
	int index1, index2, last_max_uhel, last_index1, last_index2, uhel;
	MM_DATA_STRUCT*	pts = data_arr->pts;

	last_max_uhel = 0;
	last_index1 = seg_start;
	last_index2 = seg_start;

	for (k = seg_start; k < seg_start + seg_length - 1; k++)
	{
		uhel = IsPattern2_OV(&pts[k],MIN(STEP, seg_start + seg_length - k), &index1, &index2);
		if(uhel){

			if(uhel>last_max_uhel && k <= last_index2 && last_index2){
				pts[last_index1].fix_status=0;
				pts[last_index2].fix_status=0;
			}
			if(uhel<=last_max_uhel && k <= last_index2 && last_index2){
				;
			}else{
				if(last_index2 && k > last_index2){
					if(vzd_by_degree(pts[last_index2].longitude, pts[last_index2].latitude,
							pts[k+index1].longitude, pts[k+index1].latitude)>MIN_PAT_DIST){
						pts[k+index1].fix_status=10;
						pts[k+index2].fix_status=20;
						last_index1 = index1+k;
						last_index2 = index2+k;
						last_max_uhel = uhel;
					}
				}else{
					pts[k+index1].fix_status=10;
					pts[k+index2].fix_status=20;
					last_index1 = index1+k;
					last_index2 = index2+k;
					last_max_uhel = uhel;
				}
				if(last_max_uhel>MAX_ANGLE_FOR_PAT && k< last_index2+1){
					k=last_index2+1;
				}
			}
		}
	}
}
// TMP - END


int FindLastPattern(MM_REQUEST* mm_request_2, MM_RESPONSE* mm_response_2)
{
	int last=-1, lastB=-1,lastBminus=-1,z;

	for(z=0;z<mm_request_2->gps_pts_size;z++){
		/*if(mm_response_2->map_pts[z].id == 999){
			last=z;
		}*/
		if(mm_response_2->map_pts[z].match_quality > 0){
			lastBminus=lastB;
			lastB=z;
		}

	}
	if(last == -1)
		last=lastBminus;
	return last;
}

/**********************************************************************************************************************
 * ENG: new version of mm_map_matchA_old method
 * CZE: Nova verze metody mm_map_matchA_old
 **********************************************************************************************************************/

MM_ERRORS mm_map_matchA_NV(MM_WS * ws, MM_REQUEST* mm_request, MM_RESPONSE* mm_response, int quality_requested)
{
	//MM_ERRORS result;
	MM_REQUEST mm_request_2;
	MM_MAP_PT* mm_pts;
	MM_RESPONSE mm_response_2;
	int i,z;
	MM_GPS_PT *gps_pts_input, *gps_pts;
	MM_POINT_LL route_line[100000];
	int route_line_number;
	segment seg[1000];
	//int segment_start = 0;

#ifdef DEBUG
	FILE*	fd = NULL;
	char vypis_fd[250000],tempchar[20000];
	sprintf(vypis_fd,"[");
	fd = fopen("/home/adminuser/workspace/mmnew/Debug/src/match.csv", "w");

	FILE* pm = NULL;
	char vypis_pm[250000];
	sprintf(vypis_pm,"[");
	pm = fopen("/home/adminuser/workspace/mmnew/Debug/src/pm.csv", "w");

	FILE* pat = NULL;
	char vypis_pat[250000];
	sprintf(vypis_pat,"[");
	pat = fopen("/home/adminuser/workspace/mmnew/Debug/src/pat.csv", "w");

	FILE* Frouting = NULL;
	char vypis_routing[250000];
	Frouting = fopen("/home/adminuser/workspace/mmnew/Debug/src/complete_routing.csv", "w");
	sprintf(vypis_routing, "[\n");
	fputs(vypis_routing, Frouting);
#endif	//DEBUG

#ifdef DEBUG_T8
	printf("Gps points: %i\n",mm_request->gps_pts_size);
	printf("First gps: %llu;%lld;%lld;%lld;%i;%i;%i;%i; speed:%i; heading:%i;lat:%.10f; lon%.10f\n",
			mm_request->gps_pts[0].device_id,
			(long long)mm_request->gps_pts[0].jny_start,
			(long long)mm_request->gps_pts[0].jny_end,
			(long long)mm_request->gps_pts[0].event_time,
			mm_request->gps_pts[0].RPM,
			mm_request->gps_pts[0].sats_count,
			mm_request->gps_pts[0].fix_status,
			mm_request->gps_pts[0].dist_covered,
			mm_request->gps_pts[0].speed,
			mm_request->gps_pts[0].heading,
			mm_request->gps_pts[0].latitude,
			mm_request->gps_pts[0].longitude);

#endif //DEBUG_T8

	gps_pts_input = mm_request->gps_pts;

	gps_pts = (MM_GPS_PT*)calloc(1+mm_request->gps_pts_size, sizeof(MM_GPS_PT));
	mm_pts = (MM_MAP_PT*)calloc(1+mm_request->gps_pts_size, sizeof(MM_MAP_PT));

	mm_request_2.gps_pts = gps_pts;
	mm_response_2.map_pts = mm_pts;

	// NEW
	MM_DATA data_arr;
	MM_DATA_STRUCT *data_arr_struct;

	data_arr_struct = (MM_DATA_STRUCT*)calloc(1+mm_request->gps_pts_size, sizeof(MM_DATA_STRUCT));
	data_arr.pts = data_arr_struct;

	// Convert
	Convert_MM_REQUEST2MM_DATA(mm_request, &data_arr);
	SetValidity2(&data_arr);

	// TMP
	BOOL hasGyroDiff = FALSE;
	for(i=1; i < data_arr.arr_size; i++)
	{
		if(data_arr.pts[i].gyro_diff > 0){ hasGyroDiff = TRUE; break;}
	}
	// TMP END

	if(hasGyroDiff)
	{
		CalculateGyroHeading(&data_arr);
		CopyGyro2Heading(&data_arr);
	}
	else{
		CreateGyroDif(&data_arr);
	}
	//SetSegment(ws, &data_arr);

#ifdef DEBUGM
	int r = -1;
	r = ControlPrint(mm_request, &data_arr);
	if(r == 1){}
#endif // DEBUGM



	int actual_segment_no = 0, actual_segment_start = 0, actual_segment_length = -1, j, result,last_pattern;

	do // for each segment
	{
		//result = GetSegment(&data_arr, actual_segment_no, &actual_segment_start, &actual_segment_length);
		result = CreateSegment(ws, &data_arr, &actual_segment_start, &actual_segment_length);
		if(result > 0)
		{
			for(i = actual_segment_start, j = 0; j < actual_segment_length; i++, j++)
			{
				gps_pts[j] = gps_pts_input[i];
			}

			mm_request_2.gps_pts_size = actual_segment_length;
			seg[actual_segment_no].i_start = actual_segment_start;
			seg[actual_segment_no].length = actual_segment_length;
			seg[actual_segment_no].type = 1; //good segment

			// old
			//mm_map_match_old(ws, &mm_request_2, &mm_response_2,route_line,&route_line_number);
			// new
			mm_map_match_NV(ws, &data_arr, actual_segment_start, actual_segment_length, &mm_response_2,route_line,&route_line_number);

#ifdef DEBUGM
			int ii;
			for(ii = actual_segment_start; ii < actual_segment_start + actual_segment_length; ii++)
			{
				sprintf(vypis_routing, "[%f,%f],", mm_response_2.map_pts[ii].latitude, mm_response_2.map_pts[ii].longitude);
				fputs(vypis_routing, Frouting);
			}
#endif // DEBUGM

			// old
			//findPatterns(&mm_request_2);

			// TMP
			/*BOOL hasAccelerometer = FALSE;
			for(i=1; i < data_arr.arr_size; i++)
			{
				if(data_arr.pts[i].acc_y > 0){ hasAccelerometer = TRUE; break;}
			}
			*/// TMP END

			if(/*hasAccelerometer && */hasGyroDiff)
			{
				// new
				//findPatterns_NV(&data_arr, actual_segment_start, actual_segment_length);
				// new 2
				findPatterns_NV_gyro_diff(&data_arr, actual_segment_start, actual_segment_length);
			}
			else
			{
				// old
				findPatterns_OV(&data_arr, actual_segment_start, actual_segment_length);
				//findPatterns_NV_gyro_diff(&data_arr, actual_segment_start, actual_segment_length);
			}

			// preneseni dat do stare struktury
			for(i = actual_segment_start, j = 0; j < actual_segment_length; i++, j++)
			{
				gps_pts[j].fix_status = data_arr_struct[i].fix_status;
				gps_pts[j].heading =  data_arr_struct[i].gps_heading;
			}

#ifdef DEBUGM
			for(z=0;z<mm_request_2.gps_pts_size;z++){
				if(mm_request_2.gps_pts[z].fix_status > 9){
					sprintf(tempchar,"[%f,%f],", mm_request_2.gps_pts[z].latitude, mm_request_2.gps_pts[z].longitude);
					strcat(vypis_pat,tempchar);
				}
			}
#endif	//DEBUGM

			FindPatternMatch(&mm_request_2,&mm_response_2,route_line,route_line_number);
			last_pattern = FindLastPattern(&mm_request_2,&mm_response_2);
			FillResponse(&mm_request_2,&mm_response_2, mm_response,actual_segment_start);
			FillSegment(ws,seg[actual_segment_no], mm_request, mm_response,route_line,route_line_number,quality_requested);
			clearData( &mm_request_2,  &mm_response_2);

			if(result == 2 && actual_segment_start+actual_segment_length < data_arr.arr_size){
				last_pattern -=10;
				if(last_pattern<(actual_segment_length/2))
					last_pattern=(actual_segment_length/4)*3;
				//last_pattern = MAX((actual_segment_length/10)*9,actual_segment_length-50);
				last_pattern = MAX(last_pattern,actual_segment_length-200);

				if(last_pattern < 50)
					last_pattern = actual_segment_length;
				actual_segment_start += last_pattern;
				for(i = actual_segment_start; i< seg[actual_segment_no].i_start+seg[actual_segment_no].length;i++){
					mm_response->map_pts[i].match_quality=0;
					data_arr_struct[i].fix_status = 0;
				}

				seg[actual_segment_no].length = last_pattern;
			}else
				actual_segment_start += actual_segment_length;

			actual_segment_no++;
		}
		else
		{
			// Segment doestn exist
			break;
		}
#ifdef DEBUG
	d_act_segment++;
#endif

	}while(TRUE);
	// END NEW

	//  Set QUALITY
	mm_response->quality = (1-((double)mm_response->quality / actual_segment_no)) * 100;

	int i_gps,i_seg,  last_OK, first_666,p,dp;
	double first_loc_lat_offset=0,first_loc_lon_offset=0,last_loc_lat_offset=0,last_loc_lon_offset=0 ;

	for(i_seg=0; i_seg<actual_segment_no; i_seg++)
	{
		last_OK=-1;

		first_666=-1;
		seg[i_seg].r_offset_lat = 0;
		seg[i_seg].r_offset_lon= 0;
		seg[i_seg].l_offset_lat = 0;
		seg[i_seg].l_offset_lon= 0;

		for(i_gps=seg[i_seg].i_start;i_gps<seg[i_seg].i_start+seg[i_seg].length;i_gps++)
		{

			if(mm_response->map_pts[i_gps].match_quality>0)
			{
				if(last_OK<0){
					seg[i_seg].l_offset_lat = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
					seg[i_seg].l_offset_lon= mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;
				}
				if(first_666>=0)
				{
					if(last_OK<0){
						last_loc_lat_offset = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
						last_loc_lon_offset = mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;
						first_loc_lat_offset = 0;
						first_loc_lon_offset = 0; //not ideal, ggod for long, bad for short

						seg[i_seg].l_offset_lat = 0;
						seg[i_seg].l_offset_lon= 0;

						p=i_gps-first_666;
						//dist=vzd_by_degree(mm_request->gps_pts[first_666].longitude,mm_request->gps_pts[first_666].latitude, mm_request->gps_pts[i_gps].longitude,mm_request->gps_pts[i_gps].latitude);
						for(i=first_666;i<i_gps;i++)
						{
							//dist_point = vzd_by_degree(mm_request->gps_pts[first_666].longitude,mm_request->gps_pts[first_666].latitude, mm_request->gps_pts[i].longitude,mm_request->gps_pts[i].latitude);
							//mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + first_loc_lat_offset+ ((last_loc_lat_offset-first_loc_lat_offset)*(dist_point/dist));
							//mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + first_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*(dist_point/dist));
							dp=i-i_gps;
							mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + last_loc_lat_offset+((last_loc_lat_offset-first_loc_lat_offset)*((double)dp/p));
							mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + last_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*((double)dp/p));

						}

					}
					else
					{

						first_loc_lat_offset = last_loc_lat_offset;
						first_loc_lon_offset = last_loc_lon_offset;
						last_loc_lat_offset = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
						last_loc_lon_offset = mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;

						p=i_gps-first_666;
						//lon_dist=mm_request->gps_pts[first_666-1].longitude- mm_request->gps_pts[i_gps].longitude;
						//lat_dist=mm_request->gps_pts[first_666-1].latitude- mm_request->gps_pts[i_gps].latitude;
						//dist=vzd_by_degree(mm_request->gps_pts[first_666-1].longitude,mm_request->gps_pts[first_666-1].latitude, mm_request->gps_pts[i_gps].longitude,mm_request->gps_pts[i_gps].latitude);
						for(i=first_666;i<i_gps;i++)

						{

							//lon=mm_request->gps_pts[i_gps].longitude- mm_request->gps_pts[i].longitude;
							//lat=mm_request->gps_pts[i_gps].latitude- mm_request->gps_pts[i].latitude;
							dp=i-i_gps;
							mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude +  last_loc_lat_offset+((last_loc_lat_offset-first_loc_lat_offset)*((double)dp/p));
							mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + last_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*((double)dp/p));

						}
					}
					first_666 = -1;

				}
				seg[i_seg].r_offset_lat = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
				seg[i_seg].r_offset_lon= mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;
				last_loc_lat_offset = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
				last_loc_lon_offset = mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;
				last_OK=i_gps;
			}else if(mm_response->map_pts[i_gps].match_quality==-666)
			{
				if(first_666 == -1)
					first_666=i_gps;
			}
			else // -300 a -100-x / konce
			{
				;//last_300=i_gps;
			}
		}
		if(first_666!=-1 )
		{
			first_loc_lat_offset = last_loc_lat_offset;
			first_loc_lon_offset = last_loc_lon_offset ;
			last_loc_lat_offset = 0;
			last_loc_lon_offset = 0; //not ideal, ggod for long, bad for short

			seg[i_seg].r_offset_lat = 0;
			seg[i_seg].r_offset_lon= 0;


			p=i_gps-first_666;
			//dist=vzd_by_degree(mm_request->gps_pts[first_666-1].longitude,mm_request->gps_pts[first_666-1].latitude, mm_request->gps_pts[i_gps].longitude,mm_request->gps_pts[i_gps].latitude);

			for(i=first_666;i<i_gps;i++)
			{

				if(last_OK<0)
				{
					mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude;
					mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude;
				}
				else
				{

					dp=i-i_gps;
					mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude +  last_loc_lat_offset+((last_loc_lat_offset-first_loc_lat_offset)*((double)dp/p));
					mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + last_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*((double)dp/p));

					//dist_point = vzd_by_degree(mm_request->gps_pts[first_666-1].longitude,mm_request->gps_pts[first_666-1].latitude, mm_request->gps_pts[i].longitude,mm_request->gps_pts[i].latitude);
						//mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + first_loc_lat_offset+ ((last_loc_lat_offset-first_loc_lat_offset)*(dist_point/dist));
						//mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + first_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*(dist_point/dist));
					//mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude +first_loc_lat_offset;
					//mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude +first_loc_lon_offset;

				}

			}
		}
	}

	int  last_seg_end = -1; // porovnavam s indexe dalsiho segmentu, poprve s 0

	for(i_seg=0; i_seg<actual_segment_no; i_seg++)
	{
		if((last_seg_end+1) !=seg[i_seg].i_start)
		{

			if(i_seg == 0){
					first_loc_lat_offset = seg[i_seg].l_offset_lat;
					first_loc_lon_offset = seg[i_seg].l_offset_lon;
			}else{
				if(vzd_by_degree(mm_request->gps_pts[last_seg_end].longitude,mm_request->gps_pts[last_seg_end].latitude,mm_request->gps_pts[seg[i_seg].i_start].longitude,mm_request->gps_pts[seg[i_seg].i_start].latitude)>150){
					first_loc_lat_offset = 0;
					first_loc_lon_offset = 0;
				}else{
					first_loc_lat_offset = (seg[i_seg-1].r_offset_lat+seg[i_seg].r_offset_lat)/2;
					first_loc_lon_offset = (seg[i_seg-1].r_offset_lon+seg[i_seg].r_offset_lon)/2;
				}
			}

			for(i=last_seg_end+1;i<seg[i_seg].i_start;i++)
			{
				mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + first_loc_lat_offset;
				mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude + first_loc_lon_offset;
			}
		}
		last_seg_end = seg[i_seg].i_start+seg[i_seg].length -1;
	}
	if(last_seg_end != mm_request->gps_pts_size-1){
		first_loc_lat_offset = seg[i_seg-1].r_offset_lat;
		first_loc_lon_offset = seg[i_seg-1].r_offset_lon;
		for(i=last_seg_end+1;i<mm_request->gps_pts_size;i++)
		{
			mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + first_loc_lat_offset;
			mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude+ first_loc_lon_offset;
		}
	}

	int score = 0;
	for(z=0;z<mm_request->gps_pts_size;z++){
		if(mm_response->map_pts[z].id == 999){
			mm_response->map_pts[z].match_quality = 100;
		}
		if(mm_response->map_pts[z].match_quality == 300){
			mm_response->map_pts[z].match_quality = 20;
		}
		if(mm_response->map_pts[z].match_quality == -666){
			mm_response->map_pts[z].match_quality = 0;
		}
		if(mm_response->map_pts[z].match_quality > 100){
			mm_response->map_pts[z].match_quality -= 100;
		}
		if(mm_response->map_pts[z].match_quality < 0){
			mm_response->map_pts[z].match_quality =-mm_response->map_pts[z].match_quality;
		}
		if(mm_response->map_pts[z].match_quality < 20){
			score++;
		}

	}

	if(mm_request->gps_pts_size)
		mm_response->quality=(1-((double)score/mm_request->gps_pts_size)) * 100;
	else
		mm_response->quality=100;

#ifdef DEBUG
		for(z=0;z<mm_request->gps_pts_size;z++){
			if(mm_response->map_pts[z].match_quality > 0){
				sprintf(tempchar,"[%f,%f],", mm_response->map_pts[z].latitude, mm_response->map_pts[z].longitude);
				strcat(vypis_fd,tempchar);
			}
		}

	fputs(vypis_fd,fd);
	fprintf(fd,"]\n");
	fflush(fd);
	fclose (fd);
	fputs(vypis_pm,pm);
	fprintf(pm,"]\n");
	fflush(pm);
	fclose (pm);
	fputs(vypis_pat,pat);
	fprintf(pat,"]\n");
	fflush(pat);
	fclose (pat);

	fprintf(Frouting, "\n]\n");
	fflush(Frouting);
	fclose(Frouting);
	//printf("quality %f%% %i z %i \n",(1.0-((double)mm_quality/mm_request->gps_pts_size))*100,mm_quality,mm_request->gps_pts_size);

#endif	//DEBUG
	free(gps_pts);
	free(mm_pts);

	return MM_OK;
}
/**********************************************************************************************************************
 *
 *
 *
 *
 * END
 *
 *
 *
 *
 **********************************************************************************************************************/

MM_ERRORS mm_map_matchA(MM_WS * ws, MM_REQUEST* mm_request, MM_RESPONSE* mm_response, int quality_requested)
{
	//MM_ERRORS result;
	MM_REQUEST mm_request_2;
	MM_MAP_PT* mm_pts;
	MM_RESPONSE mm_response_2;
	int i,k,dutch_count, out_count,z;
	MM_GPS_PT *gps_pts_input, *gps_pts, last_dutch,last_ptOK, temp_gps;
	MM_POINT_LL route_line[100000];
	int route_line_number;
	segment seg[1000];
	int no_seg;
	int segment_start = 0;
	//int matched;
	int dutch_count_pridavam = 0;
	int new_segment = 0;

#ifdef DEBUG
	FILE*	fd = NULL;

	char vypis_fd[100000],tempchar[20000];
	sprintf(vypis_fd,"[");
	fd = fopen("/home/adminuser/workspace/mmnew/Debug/src/match.csv", "w");

	FILE* pm = NULL;
	char vypis_pm[250000];
	sprintf(vypis_pm,"[");
	pm = fopen("/home/adminuser/workspace/mmnew/Debug/src/pm.csv", "w");

FILE* pat = NULL;
	char vypis_pat[250000];
	sprintf(vypis_pat,"[");
	pat = fopen("/home/adminuser/workspace/mmnew/Debug/src/pat.csv", "w");
#endif	//DEBUG

#ifdef DEBUG_T8
	printf("Gps points: %i\n",mm_request->gps_pts_size);
	printf("First gps: %llu;%lld;%lld;%lld;%i;%i;%i;%i; speed:%i; heading:%i;lat:%.10f; lon%.10f\n",
			mm_request->gps_pts[0].device_id,
			(long long)mm_request->gps_pts[0].jny_start,
			(long long)mm_request->gps_pts[0].jny_end,
			(long long)mm_request->gps_pts[0].event_time,
			mm_request->gps_pts[0].RPM,
			mm_request->gps_pts[0].sats_count,
			mm_request->gps_pts[0].fix_status,
			mm_request->gps_pts[0].dist_covered,
			mm_request->gps_pts[0].speed,
			mm_request->gps_pts[0].heading,
			mm_request->gps_pts[0].latitude,
			mm_request->gps_pts[0].longitude);

#endif //DEBUG_T8

	gps_pts_input = mm_request->gps_pts;

	gps_pts = (MM_GPS_PT*)calloc(1+mm_request->gps_pts_size, sizeof(MM_GPS_PT));
	mm_pts = (MM_MAP_PT*)calloc(1+mm_request->gps_pts_size, sizeof(MM_MAP_PT));

	mm_request_2.gps_pts = gps_pts;
	mm_response_2.map_pts = mm_pts;

	out_count = 0;
	dutch_count = 0;
	no_seg = 0;
	//seg[0].i_start = 0;
	last_dutch.fix_status = -1;
	last_ptOK.fix_status = -1;
	for(i=0; i < mm_request->gps_pts_size; i++)
	{
		// in case that time of next point is lower that actual point

		/***************************************************************************************************************
		 * --------------------------------------------------------------------------------------------
		 *
		 * time control is SWITCHED OFF
		 *
		 *--------------------------------------------------------------------------------------------------
		 *
		 * **********************************************************************************************************
			 if (i < mm_request->gps_pts_size - 1 && gps_pts_input[i].event_time > gps_pts_input[i+1].event_time){
			free(gps_pts);
			free(mm_pts);
			return MM_WRONG_EVENT_TIME;
		 }
*/
		if (gps_pts_input[i].speed <= MAX_SPEED_FOR_DUTCH || (gps_pts_input[i].speed <= (MAX_SPEED_FOR_DUTCH+5) && dutch_count > (MIN_COUNT_FOR_DUTCH + dutch_count_pridavam)))	// low speed
		{
			dutch_count++;
			if(/*abs(last_dutch.heading-gps_pts_input[i].heading)<10 &&*/ vzd_by_degree(last_dutch.longitude,last_dutch.latitude,gps_pts_input[i].longitude,gps_pts_input[i].latitude)<555)//25)
			{
				if(dutch_count_pridavam<60)
					dutch_count_pridavam++;
			}

			if(last_dutch.fix_status == -1)
					last_dutch = gps_pts_input[i];

			// split too distanced FD into two groups
			if (i < mm_request->gps_pts_size - 1 && (abs(gps_pts_input[i+1].event_time - gps_pts_input[i].event_time) > FD_MAX_SPACE_BETWEEN_POINTS ||
					vzd_by_degree( gps_pts_input[i+1].longitude,gps_pts_input[i+1].latitude,gps_pts_input[i].longitude,gps_pts_input[i].latitude) > FD_MAX_DIST_BETWEEN_POINTS)){


				new_segment = 1;
				dutch_count = 0;
				dutch_count_pridavam = 0;
				last_dutch.fix_status = -1; //?
			}


			if (dutch_count >= (MIN_COUNT_FOR_DUTCH + dutch_count_pridavam) || new_segment){	// flying dutch found
				new_segment = 0;
				if (out_count > 1) {		//there is enough points for processing
					//mm_request_2.gps_pts_size = out_count;



					mm_request_2.gps_pts_size = find_reasonable_count(ws, IMM_OSM, mm_request_2.gps_pts, out_count);
					mm_map_match_old(ws, &mm_request_2, &mm_response_2,route_line,&route_line_number);
					seg[no_seg].i_start=segment_start;
					seg[no_seg].length=mm_request_2.gps_pts_size;
					seg[no_seg].type = 1; //good segment
					findPatterns(&mm_request_2);
					FindPatternMatch(&mm_request_2,&mm_response_2,route_line,route_line_number);
					FillResponse(&mm_request_2,&mm_response_2, mm_response,segment_start);
					FillSegment(ws,seg[no_seg], mm_request, mm_response,route_line,route_line_number,quality_requested);
					no_seg++;

					if (mm_request_2.gps_pts_size != out_count)	{
						i=i-out_count-dutch_count+mm_request_2.gps_pts_size-1;
						dutch_count=0;
					}

#ifdef DEBUG
					for(z=0;z<mm_request_2.gps_pts_size ;z++){
						if(mm_request_2.gps_pts[z].fix_status > 9){
						sprintf(tempchar,"[%f,%f],", mm_request_2.gps_pts[z].latitude, mm_request_2.gps_pts[z].longitude);
						strcat(vypis_pat,tempchar);
						}
					}
#endif	//DEBUG
					clearData( &mm_request_2,  &mm_response_2);
					out_count = 0;
				}
				else
				{
					out_count = 0;		// only one point in out data, lets start over
				}
			}
		}
		else
		{
			if (dutch_count != 0) {	// dutch points

				if (dutch_count >= MIN_COUNT_FOR_DUTCH + dutch_count_pridavam || i == dutch_count)
				{
					out_count = 0;
					dutch_count = 0;
				}

				for(k=dutch_count; k > 0; k--)
				{
					if(out_count == 0)
						segment_start=i-dutch_count;
					gps_pts[out_count] = gps_pts_input[i-k];
#ifdef DEBUG
					//	sprintf(tempchar,"[%f,%f],", gps_pts_input[i-k].latitude, gps_pts_input[i-k].longitude);
					//	strcat(vypis_pm,tempchar);
#endif	//DEBUG

					out_count++;
				}
				dutch_count_pridavam = 0;
				dutch_count = 0;
				last_dutch.fix_status = -1;

			}
			if(last_ptOK.fix_status!=-1){
				temp_gps = last_ptOK;
			}else{
				if( i != 0)
					temp_gps=gps_pts_input[i-1];
			}


			if (out_count != 0 && i != 0 && (abs(temp_gps.event_time - gps_pts_input[i].event_time) > FD_MAX_SPACE_BETWEEN_POINTS ||
				vzd_by_degree( temp_gps.longitude,temp_gps.latitude,gps_pts_input[i].longitude,gps_pts_input[i].latitude) > FD_MAX_DIST_BETWEEN_POINTS))
			{
				mm_request_2.gps_pts_size = find_reasonable_count(ws, IMM_OSM, mm_request_2.gps_pts, out_count);
				mm_map_match_old(ws, &mm_request_2, &mm_response_2,route_line,&route_line_number);
				seg[no_seg].i_start=segment_start;
				seg[no_seg].length=mm_request_2.gps_pts_size;
				seg[no_seg].type = 1; //good segment
				findPatterns(&mm_request_2);
				FindPatternMatch(&mm_request_2,&mm_response_2,route_line,route_line_number);
				FillResponse(&mm_request_2,&mm_response_2, mm_response,segment_start);
				FillSegment(ws, seg[no_seg], mm_request, mm_response,route_line,route_line_number,quality_requested);
				no_seg++;

				if (mm_request_2.gps_pts_size != out_count)	{
					i=i-out_count-dutch_count+mm_request_2.gps_pts_size;
				}

#ifdef DEBUG
				for(z=0;z<mm_request_2.gps_pts_size;z++){
					if(mm_request_2.gps_pts[z].fix_status > 9){
					sprintf(tempchar,"[%f,%f],", mm_request_2.gps_pts[z].latitude, mm_request_2.gps_pts[z].longitude);
					strcat(vypis_pat,tempchar);
					}
				}
#endif	//DEBUG

				clearData( &mm_request_2,  &mm_response_2);
				out_count = 0;
				dutch_count_pridavam = 0;
				dutch_count=0;
				last_dutch.fix_status = -1;

			}
			if(out_count == 0)
				segment_start=i;
			gps_pts[out_count] = gps_pts_input[i];
#ifdef DEBUG
						//sprintf(tempchar,"[%f,%f],", gps_pts_input[i].latitude, gps_pts_input[i].longitude);
						//strcat(vypis_pm,tempchar);
#endif	//DEBUG
			out_count++;
			last_ptOK=gps_pts_input[i];
		}

		if (i == (mm_request->gps_pts_size-1) && out_count > 1) {		// last point


			/*
			 * Resi konec FD
			if (dutch_count != 0) {	// dutch points

					for(k=dutch_count; k > 0; k--)
					{
						if(out_count == 0)
							segment_start=i-dutch_count;
						gps_pts[out_count] = gps_pts_input[i-k];
	#ifdef DEBUG
							sprintf(tempchar,"[%f,%f],", gps_pts_input[i-k].latitude, gps_pts_input[i-k].longitude);
							strcat(vypis_pm,tempchar);
	#endif	//DEBUG
						out_count++;
					}
					dutch_count_pridavam = 0;
					dutch_count = 0;
					last_dutch.fix_status = -1;

			}
			*/


			mm_request_2.gps_pts_size = find_reasonable_count(ws, IMM_OSM, mm_request_2.gps_pts, out_count);
			mm_map_match_old(ws, &mm_request_2, &mm_response_2,route_line,&route_line_number);
			seg[no_seg].i_start=segment_start;
			seg[no_seg].length=mm_request_2.gps_pts_size;
			seg[no_seg].type = 1; //good segment
			findPatterns(&mm_request_2);
			FindPatternMatch(&mm_request_2,&mm_response_2,route_line,route_line_number);
			FillResponse(&mm_request_2,&mm_response_2, mm_response,segment_start);
			FillSegment(ws,seg[no_seg], mm_request, mm_response,route_line,route_line_number,quality_requested);
			no_seg++;

			if (mm_request_2.gps_pts_size != out_count)	{
				i=i-out_count-dutch_count+mm_request_2.gps_pts_size;
			}
#ifdef DEBUG
		for(z=0;z<mm_request_2.gps_pts_size;z++){
			if(mm_request_2.gps_pts[z].fix_status > 9){
				sprintf(tempchar,"[%f,%f],", mm_request_2.gps_pts[z].latitude, mm_request_2.gps_pts[z].longitude);
				strcat(vypis_pat,tempchar);
			}
		}
#endif	//DEBUG

		clearData( &mm_request_2,  &mm_response_2);

			out_count = 0;
			dutch_count_pridavam = 0;
			dutch_count=0;
			last_dutch.fix_status = -1;

		}

	}

int i_gps,i_seg,  last_OK, first_666,p,dp;
double first_loc_lat_offset=0,first_loc_lon_offset=0,last_loc_lat_offset=0,last_loc_lon_offset=0 ;

	for(i_seg=0;i_seg<no_seg; i_seg++)
	{
		last_OK=-1;

		first_666=-1;
		seg[i_seg].r_offset_lat = 0;
		seg[i_seg].r_offset_lon= 0;
		seg[i_seg].l_offset_lat = 0;
		seg[i_seg].l_offset_lon= 0;

		for(i_gps=seg[i_seg].i_start;i_gps<seg[i_seg].i_start+seg[i_seg].length;i_gps++)
		{

			if(mm_response->map_pts[i_gps].match_quality>0)
			{
				if(last_OK<0){
					seg[i_seg].l_offset_lat = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
					seg[i_seg].l_offset_lon= mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;
				}
				if(first_666>=0)
				{
					if(last_OK<0){
						last_loc_lat_offset = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
						last_loc_lon_offset = mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;
						first_loc_lat_offset = 0;
						first_loc_lon_offset = 0; //not ideal, ggod for long, bad for short

						seg[i_seg].l_offset_lat = 0;
						seg[i_seg].l_offset_lon= 0;

						p=i_gps-first_666;
						//dist=vzd_by_degree(mm_request->gps_pts[first_666].longitude,mm_request->gps_pts[first_666].latitude, mm_request->gps_pts[i_gps].longitude,mm_request->gps_pts[i_gps].latitude);
						for(i=first_666;i<i_gps;i++)
						{
							//dist_point = vzd_by_degree(mm_request->gps_pts[first_666].longitude,mm_request->gps_pts[first_666].latitude, mm_request->gps_pts[i].longitude,mm_request->gps_pts[i].latitude);
							//mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + first_loc_lat_offset+ ((last_loc_lat_offset-first_loc_lat_offset)*(dist_point/dist));
							//mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + first_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*(dist_point/dist));
							dp=i-i_gps;
							mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + last_loc_lat_offset+((last_loc_lat_offset-first_loc_lat_offset)*((double)dp/p));
							mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + last_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*((double)dp/p));

						}

					}
					else
					{

						first_loc_lat_offset = last_loc_lat_offset;
						first_loc_lon_offset = last_loc_lon_offset;
						last_loc_lat_offset = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
						last_loc_lon_offset = mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;

						p=i_gps-first_666;
						//lon_dist=mm_request->gps_pts[first_666-1].longitude- mm_request->gps_pts[i_gps].longitude;
						//lat_dist=mm_request->gps_pts[first_666-1].latitude- mm_request->gps_pts[i_gps].latitude;
						//dist=vzd_by_degree(mm_request->gps_pts[first_666-1].longitude,mm_request->gps_pts[first_666-1].latitude, mm_request->gps_pts[i_gps].longitude,mm_request->gps_pts[i_gps].latitude);
						for(i=first_666;i<i_gps;i++)

						{

							//lon=mm_request->gps_pts[i_gps].longitude- mm_request->gps_pts[i].longitude;
							//lat=mm_request->gps_pts[i_gps].latitude- mm_request->gps_pts[i].latitude;
							dp=i-i_gps;
							mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude +  last_loc_lat_offset+((last_loc_lat_offset-first_loc_lat_offset)*((double)dp/p));
							mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + last_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*((double)dp/p));

						}
					}
					first_666 = -1;

				}
				seg[i_seg].r_offset_lat = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
				seg[i_seg].r_offset_lon= mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;
				last_loc_lat_offset = mm_response->map_pts[i_gps].latitude - mm_request->gps_pts[i_gps].latitude;
				last_loc_lon_offset = mm_response->map_pts[i_gps].longitude - mm_request->gps_pts[i_gps].longitude;
				last_OK=i_gps;
			}else if(mm_response->map_pts[i_gps].match_quality==-666)
			{
				if(first_666 == -1)
					first_666=i_gps;
			}
			else // -300 a -100-x / konce
			{
				;//last_300=i_gps;
			}
		}
		if(first_666!=-1 )
		{
			first_loc_lat_offset = last_loc_lat_offset;
			first_loc_lon_offset = last_loc_lon_offset ;
			last_loc_lat_offset = 0;
			last_loc_lon_offset = 0; //not ideal, ggod for long, bad for short

			seg[i_seg].r_offset_lat = 0;
			seg[i_seg].r_offset_lon= 0;


			p=i_gps-first_666;
			//dist=vzd_by_degree(mm_request->gps_pts[first_666-1].longitude,mm_request->gps_pts[first_666-1].latitude, mm_request->gps_pts[i_gps].longitude,mm_request->gps_pts[i_gps].latitude);

			for(i=first_666;i<i_gps;i++)
			{

				if(last_OK<0)
				{
					mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude;
					mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude;
				}
				else
				{

					dp=i-i_gps;
					mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude +  last_loc_lat_offset+((last_loc_lat_offset-first_loc_lat_offset)*((double)dp/p));
					mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + last_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*((double)dp/p));

					//dist_point = vzd_by_degree(mm_request->gps_pts[first_666-1].longitude,mm_request->gps_pts[first_666-1].latitude, mm_request->gps_pts[i].longitude,mm_request->gps_pts[i].latitude);
						//mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + first_loc_lat_offset+ ((last_loc_lat_offset-first_loc_lat_offset)*(dist_point/dist));
						//mm_response->map_pts[i].longitude= mm_request->gps_pts[i].longitude + first_loc_lon_offset+((last_loc_lon_offset-first_loc_lon_offset)*(dist_point/dist));
					//mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude +first_loc_lat_offset;
					//mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude +first_loc_lon_offset;

				}

			}
		}
	}

	int  last_seg_end = -1; // porovnavam s indexe dalsiho segmentu, poprve s 0

	for(i_seg=0;i_seg<no_seg; i_seg++)
	{
		if((last_seg_end+1) !=seg[i_seg].i_start)
		{

			if(i_seg == 0){
					first_loc_lat_offset = seg[i_seg].l_offset_lat;
					first_loc_lon_offset = seg[i_seg].l_offset_lon;
			}else{
				if(vzd_by_degree(mm_request->gps_pts[last_seg_end].longitude,mm_request->gps_pts[last_seg_end].latitude,mm_request->gps_pts[seg[i_seg].i_start].longitude,mm_request->gps_pts[seg[i_seg].i_start].latitude)>150){
					first_loc_lat_offset = 0;
					first_loc_lon_offset = 0;
				}else{
					first_loc_lat_offset = (seg[i_seg-1].r_offset_lat+seg[i_seg].r_offset_lat)/2;
					first_loc_lon_offset = (seg[i_seg-1].r_offset_lon+seg[i_seg].r_offset_lon)/2;
				}
			}

			for(i=last_seg_end+1;i<seg[i_seg].i_start;i++)
			{
				mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + first_loc_lat_offset;
				mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude + first_loc_lon_offset;
			}
		}
		last_seg_end = seg[i_seg].i_start+seg[i_seg].length -1;
	}
	if(last_seg_end != mm_request->gps_pts_size-1){
		first_loc_lat_offset = seg[i_seg-1].r_offset_lat;
		first_loc_lon_offset = seg[i_seg-1].r_offset_lon;
		for(i=last_seg_end+1;i<mm_request->gps_pts_size;i++)
		{
			mm_response->map_pts[i].latitude = mm_request->gps_pts[i].latitude + first_loc_lat_offset;
			mm_response->map_pts[i].longitude = mm_request->gps_pts[i].longitude+ first_loc_lon_offset;
		}
	}

	int score = 0;
	for(z=0;z<mm_request->gps_pts_size;z++){
		if(mm_response->map_pts[z].id == 999){
			mm_response->map_pts[z].match_quality = 100;
		}
		if(mm_response->map_pts[z].match_quality == 300){
			mm_response->map_pts[z].match_quality = 20;
		}
		if(mm_response->map_pts[z].match_quality == -666){
			mm_response->map_pts[z].match_quality = 0;
		}
		if(mm_response->map_pts[z].match_quality > 100){
			mm_response->map_pts[z].match_quality -= 100;
		}
		if(mm_response->map_pts[z].match_quality < 0){
			mm_response->map_pts[z].match_quality =-mm_response->map_pts[z].match_quality;
		}
		if(mm_response->map_pts[z].match_quality < 20){
			score++;
		}

	}

	if(mm_request->gps_pts_size)
		mm_response->quality=(1-((double)score/mm_request->gps_pts_size)) * 100;
	else
		mm_response->quality=100;

#ifdef DEBUG
		for(z=0;z<mm_request->gps_pts_size;z++){
			if(mm_response->map_pts[z].match_quality > 0){
				sprintf(tempchar,"[%f,%f],", mm_response->map_pts[z].latitude, mm_response->map_pts[z].longitude);
				strcat(vypis_fd,tempchar);
			}
		}

	fputs(vypis_fd,fd);
	fprintf(fd,"]\n");
	fflush(fd);
	fclose (fd);
	fputs(vypis_pm,pm);
	fprintf(pm,"]\n");
	fflush(pm);
	fclose (pm);
	fputs(vypis_pat,pat);
	fprintf(pat,"]\n");
	fflush(pat);
	fclose (pat);
	//printf("quality %f%% %i z %i \n",(1.0-((double)mm_quality/mm_request->gps_pts_size))*100,mm_quality,mm_request->gps_pts_size);

#endif	//DEBUG
	free(gps_pts);
	free(mm_pts);

	return MM_OK;
}


MM_ERRORS mm_map_match(MM_WS * ws, MM_REQUEST* mm_request, MM_RESPONSE* mm_response){

	int ret_code,i;

	MM_REQUEST mm_request_2;
	MM_GPS_PT *gps_pts;

	gps_pts = (MM_GPS_PT*)calloc(mm_request->gps_pts_size, sizeof(MM_GPS_PT));

	mm_request_2.gps_pts = gps_pts;
	mm_request_2.gps_pts_size = mm_request->gps_pts_size;

	// original
	//ret_code = mm_map_matchA(ws, mm_request, mm_response,50);

	// new
	ret_code = mm_map_matchA_NV(ws, mm_request, mm_response,50); //50


	if(ret_code == MM_OK){
     for(i=0;i<mm_request->gps_pts_size ;i++){
		mm_request_2.gps_pts[i]=mm_request->gps_pts[i];
		mm_request_2.gps_pts[i].latitude = mm_response->map_pts[i].latitude;
		mm_request_2.gps_pts[i].longitude = mm_response->map_pts[i].longitude;
		mm_response->map_pts[i].latitude = 0;
		mm_response->map_pts[i].longitude = 0;
		mm_response->map_pts[i].id = 0;
		mm_response->map_pts[i].match_quality = 0;
	 }

     // original
     //ret_code = mm_map_matchA(ws, &mm_request_2, mm_response,10);

     // new
     ret_code = mm_map_matchA_NV(ws, &mm_request_2, mm_response,10);
	}

	free(gps_pts);
	return (MM_ERRORS)ret_code;
}




void imm_close_unit(IMM_WS_UNIT* ws)
{
	if(((IMM_CACHE*)((ws)->imm_cache))->gps_cache.gps_pts != NULL)
		free(((IMM_CACHE*)((ws)->imm_cache))->gps_cache.gps_pts);
	free(ws->imm_cache);
	free(ws);
}
void imm_close_db(IMM_WS_DB* ws)
{
	xdb_close(&ws->xdb);
	free(ws);
}
int imm_init_unit(IMM_INIT_UNIT *init_data, IMM_WS_UNIT** workspace)	///HERE///
{

	if( NULL == (*workspace = (IMM_WS_UNIT*)malloc(sizeof(IMM_WS_UNIT))) )	// memory is freed in mm_close method
	{
		return 1;
	}
	if( NULL == ((*workspace)->imm_cache = (IMM_CACHE*)malloc(sizeof(IMM_CACHE))) )	// memory is freed in imm_close_unit method
	{
		return 1;
	}
	if( NULL == (((IMM_CACHE*)((*workspace)->imm_cache))->gps_cache.gps_pts = (IMM_GPS_PT*)calloc(MAX_GPS_CACHE, sizeof(IMM_GPS_PT)))) // allocate memory for all GPS point in cache + in request
	{
		((IMM_CACHE*)((*workspace)->imm_cache))->gps_cache.gps_pts = NULL;
	}
	((IMM_CACHE*)((*workspace)->imm_cache))->gps_cache.gps_pts_size = 0;

	((IMM_CACHE*)((*workspace)->imm_cache))->map_source = init_data->map_source;
	((IMM_CACHE*)((*workspace)->imm_cache))->algorithm = init_data->algorithm;

	return 0;
}
int imm_init_db(IMM_INIT_DB *init_data, IMM_WS_DB** workspace)
{
	xdb_t*	xdb;

	if( NULL == (*workspace = (IMM_WS_DB*)malloc(sizeof(IMM_WS_DB))) )	// memory is freed in imm_close_db method
	{
		return 1;
	}

	xdb = &((*workspace)->xdb);

	//Init DB.
	xdb_init2(xdb, init_data->host, init_data->user, init_data->passwd, init_data->db, init_data->port);

	return 0;
}
int mm_get_postcode(IMM_WS_DB * ws_db, double lon, double lat, char* post_code)
{
	char query[4096];
	const char *query_ptr = query;
	xdb_t*	xdb;
	xdb_result_t *xdb_result;
	int db_index;
	char *actual_postcode = NULL;

	xdb_result = NULL;
	xdb = &(ws_db->xdb);
	db_index = 0;

    sprintf(query,
                    "SELECT postcode,"
                    " SQRT(((RADIANS((latitude  - %lf ))  * 6370) * (RADIANS((latitude - %lf)) * 6370)) + "
                    " ((RADIANS((longitude  - %lf)) * 6370  *  COS(RADIANS(%lf)))    * "
                    " (RADIANS((longitude  - %lf)) * 6370  *  COS(RADIANS(%lf)))))   * 1000 AS DISTANCE "
                    " FROM `%s`"
                    " WHERE    Within(PointFromText('POINT(%lf %lf)'), border) "
                    " ORDER    BY DISTANCE ASC limit 1 ",
                    lat, lat, lon, lat, lon, lat, POSTCODES_TABLE, lon, lat);


	//printf("%s\n",query);

	xdb_result = xdb_query(xdb, query_ptr);

	if (xdb_result == NULL)
	{
		return MM_NO_DB_RESPONSE;
	}
	else
	{

		xdb_get_next_row(xdb_result);
		if (xdb_result->row_count != 0){
			xdb_get_field_string(xdb_result, db_index++, &actual_postcode);
			if (actual_postcode){
				sprintf(post_code,"%s", actual_postcode);
				free(actual_postcode);
				}
		}

		xdb_free_result(xdb_result);

		return MM_OK;
	}
}
int get_additional_data(IMM_WS_DB *ws_db, int gps_pts_size, IMM_MAP_PT* map_pts)
{
	char query[4096];
	char ids[4096];
	char id[256];
	UINT last_id;
	int i,db_index, actual_point = 0;
	xdb_t*	xdb;
	xdb_result_t *xdb_result = NULL;
	const char *query_ptr = query;
	double *avg_speeds;

	int link_id = 0;
	int fr_spd_lim,to_spd_lim,speed_cat,route_type;
	char *route_name,*street_name_native,*street_name_transcription,*admin_area1_name_native,*admin_area1_name_transcription;
	char *admin_area2_name_native,*admin_area2_name_transcription,*admin_area4_name_native,*admin_area4_name_transcription;
	char *l_postcode,*r_postcode,*time_zone, *urban, *spd_lim_unit, *tollway;
	int func_class;
	int national_speed_limit;
	BOOL speedUnclear;

	sprintf(ids, "%d", map_pts[0].id);	// get first id
	last_id = map_pts[0].id;

	for(i=1; i < gps_pts_size ;i++)		// get rest of id's
	{
		if (map_pts[i].id != last_id){
			sprintf(id, ",%d", map_pts[i].id);
			strcat(ids,id);
			last_id = map_pts[i].id;
		}
	}

	sprintf(query, "SELECT %s,fr_spd_lim,to_spd_lim,speed_cat,route_name,street_name_native,street_name_transcription"
			",admin_area1_name_native,admin_area1_name_transcription,admin_area2_name_native,admin_area2_name_transcription"
			",admin_area4_name_native,admin_area4_name_transcription"
			",l_postcode,r_postcode,time_zone,func_class,national_speed_limit,urban,speed_limit_unit "
			",route_type,tollway "
			"FROM %s WHERE %s IN (%s)",
			HERE_ID_ATTRIBUTE,HERE_TABLE,HERE_ID_ATTRIBUTE,ids);

	///printf("%s\n",query);

	xdb = &(ws_db->xdb);
	xdb_result = NULL;
	xdb_result = xdb_query(xdb, query_ptr);

	route_name = NULL;
	street_name_native = NULL;
	street_name_transcription = NULL;
	admin_area1_name_native = NULL;
	admin_area1_name_transcription=NULL;
	admin_area2_name_native = NULL;
	admin_area2_name_transcription = NULL;
	admin_area4_name_native = NULL;
	admin_area4_name_transcription = NULL;
	l_postcode = NULL;
	r_postcode = NULL;
	time_zone = NULL;
	urban = NULL;
	spd_lim_unit = NULL;
	func_class = 0;
	national_speed_limit = 0;
	tollway = NULL;

	while(xdb_get_next_row(xdb_result))
	{
		db_index = 0;

		if (xdb_get_field_int(xdb_result, db_index++, &link_id) == 0)
			return 1;
		if (xdb_get_field_int(xdb_result, db_index++, &fr_spd_lim) == 0)
			return 1;
		if (xdb_get_field_int(xdb_result, db_index++, &to_spd_lim) == 0)
			return 1;
		if (xdb_get_field_int(xdb_result, db_index++, &speed_cat) == 0)
			return 1;

		xdb_get_field_string(xdb_result, db_index++, &route_name);
		xdb_get_field_string(xdb_result, db_index++, &street_name_native);
		xdb_get_field_string(xdb_result, db_index++, &street_name_transcription);
		xdb_get_field_string(xdb_result, db_index++, &admin_area1_name_native);
		xdb_get_field_string(xdb_result, db_index++, &admin_area1_name_transcription);
		xdb_get_field_string(xdb_result, db_index++, &admin_area2_name_native);
		xdb_get_field_string(xdb_result, db_index++, &admin_area2_name_transcription);
		//xdb_get_field_string(xdb_result, db_index++, &admin_area3_name_native);
		//xdb_get_field_string(xdb_result, db_index++, &admin_area3_name_transcription);
		xdb_get_field_string(xdb_result, db_index++, &admin_area4_name_native);
		xdb_get_field_string(xdb_result, db_index++, &admin_area4_name_transcription);
		//xdb_get_field_string(xdb_result, db_index++, &admin_area5_name_native);
		//xdb_get_field_string(xdb_result, db_index++, &admin_area5_name_transcription);
		xdb_get_field_string(xdb_result, db_index++, &l_postcode);
		xdb_get_field_string(xdb_result, db_index++, &r_postcode);
		xdb_get_field_string(xdb_result, db_index++, &time_zone);

		xdb_get_field_int(xdb_result, db_index++, &func_class);
		xdb_get_field_int(xdb_result, db_index++, &national_speed_limit);

		xdb_get_field_string(xdb_result, db_index++, &urban);
		xdb_get_field_string(xdb_result, db_index++, &spd_lim_unit);
		xdb_get_field_int(xdb_result, db_index++, &route_type);
		xdb_get_field_string(xdb_result, db_index++, &tollway);

		for(actual_point=0; actual_point < gps_pts_size ;actual_point++)
		{

			if(map_pts[actual_point].speed_limit >0)
				speedUnclear = FALSE;
			else
				speedUnclear = TRUE;

			//while(map_pts[actual_point].id == link_id)
			if (map_pts[actual_point].id == link_id)
			{
				//A zero value indicates that both strings are equal.
				if (spd_lim_unit && !strcmp(spd_lim_unit, "MPH"))
				{
					avg_speeds = AVG_M_PER_S_FROM_MPH;
					//max_speeds = MAX_M_PER_S_FROM_MPH;
					mm_get_postcode(ws_db, map_pts[actual_point].longitude, map_pts[actual_point].latitude, map_pts[actual_point].post_code);
					if (map_pts[actual_point].post_code[0] == '\0'){
						if (r_postcode)
							sprintf(map_pts[actual_point].post_code,"%s",r_postcode);
						else if (l_postcode)
							sprintf(map_pts[actual_point].post_code,"%s",l_postcode);
					}

					if(speedUnclear == FALSE)
						map_pts[actual_point].speed_limit = (fr_spd_lim > to_spd_lim) ? MPH_TO_M_PER_S(fr_spd_lim) : MPH_TO_M_PER_S(to_spd_lim);
					else
						map_pts[actual_point].speed_limit = 0;
					//						map_pts[actual_point].speed_limit = MPH_TO_M_PER_S(national_speed_limit);
				}
				else
				{
					avg_speeds = AVG_M_PER_S_FROM_KPH;
					//max_speeds = MAX_M_PER_S_FROM_KPH;

					///if (!strcmp(admin_area1_name_native, "Ireland"))
					///mm_get_postcode(ws_db, map_pts[actual_point].longitude, map_pts[actual_point].latitude, map_pts[actual_point].post_code);
					if (map_pts[actual_point].post_code[0] == '\0'){
						if (r_postcode)
							sprintf(map_pts[actual_point].post_code,"%s",r_postcode);
						else if (l_postcode)
							sprintf(map_pts[actual_point].post_code,"%s",l_postcode);
					}

					if(speedUnclear == FALSE)
						map_pts[actual_point].speed_limit = (fr_spd_lim > to_spd_lim) ? KPH_TO_M_PER_S(fr_spd_lim) : KPH_TO_M_PER_S(to_spd_lim);
					else
						map_pts[actual_point].speed_limit = 0;
					//					if (map_pts[actual_point].speed_limit == 0)
					//						map_pts[actual_point].speed_limit = KPH_TO_M_PER_S(national_speed_limit);
				}


				switch (speed_cat)
				{
				case 1: map_pts[actual_point].average_speed = (map_pts[actual_point].speed_limit == 0) ? avg_speeds[7] : MIN(avg_speeds[7],map_pts[actual_point].speed_limit);
				break;
				case 2: map_pts[actual_point].average_speed = (map_pts[actual_point].speed_limit == 0) ? avg_speeds[6] : MIN(avg_speeds[6],map_pts[actual_point].speed_limit);
				break;
				case 3: map_pts[actual_point].average_speed = (map_pts[actual_point].speed_limit == 0) ? avg_speeds[5] : MIN(avg_speeds[5],map_pts[actual_point].speed_limit);
				break;
				case 4: map_pts[actual_point].average_speed = (map_pts[actual_point].speed_limit == 0) ? avg_speeds[4] : MIN(avg_speeds[4],map_pts[actual_point].speed_limit);
				break;
				case 5: map_pts[actual_point].average_speed = (map_pts[actual_point].speed_limit == 0) ? avg_speeds[3] : MIN(avg_speeds[3],map_pts[actual_point].speed_limit);
				break;
				case 6: map_pts[actual_point].average_speed = (map_pts[actual_point].speed_limit == 0) ? avg_speeds[2] : MIN(avg_speeds[2],map_pts[actual_point].speed_limit);
				break;
				case 7: map_pts[actual_point].average_speed = (map_pts[actual_point].speed_limit == 0) ? avg_speeds[1] : MIN(avg_speeds[1],map_pts[actual_point].speed_limit);
				break;
				case 8: map_pts[actual_point].average_speed = (map_pts[actual_point].speed_limit == 0) ? avg_speeds[0] : MIN(avg_speeds[0],map_pts[actual_point].speed_limit);
				break;

				default: //TODO
					break;
				}

				if (route_name)
					sprintf(map_pts[actual_point].route_name,"%s",route_name);
				if (street_name_native)
					sprintf(map_pts[actual_point].street_name,"%s",street_name_native);
				if (street_name_transcription)
					sprintf(map_pts[actual_point].street_name_latin,"%s",street_name_transcription);

				if (admin_area1_name_native)
					sprintf(map_pts[actual_point].country_name,"%s",admin_area1_name_native);	//COUNTRY
				if (admin_area1_name_transcription)
					sprintf(map_pts[actual_point].country_name_latin,"%s",admin_area1_name_transcription);

				if (admin_area2_name_native)
					sprintf(map_pts[actual_point].county_name,"%s",admin_area2_name_native);	//COUNTY
				if (admin_area2_name_transcription)
					sprintf(map_pts[actual_point].county_name_latin,"%s",admin_area2_name_transcription);

				//if (urban[0] == 'Y'){
				if (admin_area4_name_native)
					sprintf(map_pts[actual_point].town_name,"%s",admin_area4_name_native);	//TOWN
				if (admin_area4_name_transcription)
					sprintf(map_pts[actual_point].town_name_latin,"%s",admin_area4_name_transcription);
				//}

				if (time_zone)
					sprintf(map_pts[actual_point].time_zone,"%s",time_zone);

				map_pts[actual_point].speed_cat = speed_cat;
				map_pts[actual_point].func_class = func_class;
				map_pts[actual_point].route_type = route_type;
				map_pts[actual_point].tollway = 0;

				if (tollway && tollway[0] == 'Y')
					map_pts[actual_point].tollway = 1;

				///actual_point++;
				if (actual_point == gps_pts_size)	//last point reached
					break;
			}
		}

		if (route_name){
			free(route_name);
			route_name = NULL;
		}
		if (street_name_native){
			free(street_name_native);
			street_name_native = NULL;
		}
		if (street_name_transcription){
			free(street_name_transcription);
			street_name_transcription = NULL;
		}
		if (admin_area1_name_native){
			free(admin_area1_name_native);
			admin_area1_name_native = NULL;
		}
		if (admin_area1_name_transcription){
			free(admin_area1_name_transcription);
			admin_area1_name_transcription=NULL;
		}
		if (admin_area2_name_native){
			free(admin_area2_name_native);
			admin_area2_name_native = NULL;
		}
		if (admin_area2_name_transcription){
			free(admin_area2_name_transcription);
			admin_area2_name_transcription = NULL;
		}
		if (admin_area4_name_native){
			free(admin_area4_name_native);
			admin_area4_name_native = NULL;
		}
		if (admin_area4_name_transcription){
			free(admin_area4_name_transcription);
			admin_area4_name_transcription = NULL;
		}
		if (l_postcode){
			free(l_postcode);
			l_postcode = NULL;
		}
		if (r_postcode){
			free(r_postcode);
			r_postcode = NULL;
		}
		if (time_zone){
			free(time_zone);
			time_zone = NULL;
		}
		if (urban){
			free(urban);
			urban = NULL;
		}
		if (spd_lim_unit){
			free(spd_lim_unit);
			spd_lim_unit = NULL;
		}
		if (tollway){
			free(tollway);
			tollway = NULL;
		}
		func_class = 0;
		national_speed_limit = 0;
	}

	xdb_free_result(xdb_result);

	return 0;
}

int HeadingsDifferenceAngles(int heading1, int heading2) {
	int result;
	//result = abs(heading1-heading2);
	//if(result > 180) {
	//	result = 360 - result;
	//}
	result = heading2-heading1;
	if(result > 180) {
		result = 360 - result;
		result = -result;
	}else{
		if(result < -180) {
			result = 360 + result;
		}
	}

	return result;
}
static int SnapToLine(MM_WKBLINE *line, double gps_lon, double gps_lat, double *lon_snap, double *lat_snap, double *shortest_distance, double* dist_to_start,	double degree_coef,	int *heading)

{
	double distance = DBL_MAX; ///< current distance between GPS location and candidate solution
    double d_mm = 0.0; /// distance from start to current part

    double p_lon, p_lat, /*lon_snap_degraded,*/ v_lon, v_lat, w_lon, w_lat, res_lon, res_lat;
    int i = 0;

    *shortest_distance = distance; ///< candidate distance

    if(line->num_points < 2) {
        return -1;
    }

    // For each pair of points
    for(i = 0; i<line->num_points-1; i++)
    {
    	p_lon = gps_lon*degree_coef;
    	p_lat = gps_lat;
    	v_lon = line->points[i].x*degree_coef;
    	v_lat = line->points[i].y;
    	w_lon = line->points[i+1].x*degree_coef;
    	w_lat = line->points[i+1].y;
    	pointOnSegment(p_lon, p_lat, v_lon, v_lat, w_lon, w_lat, &res_lon, &res_lat);
	res_lon = res_lon / degree_coef;

        distance = vzd_by_degree(gps_lon,gps_lat, res_lon, res_lat);
        if(distance < *shortest_distance) {
            // Found a better solution, use this one
            *lon_snap = res_lon;
            *lat_snap = res_lat;
            *shortest_distance = distance;
            *heading = 90- (atan2(w_lat - v_lat, (w_lon - v_lon)) * (180.0 / M_PI));// heading from box has 0 no north and clockwise
        	if (*heading <0){
        		*heading = *heading +360;
        	}
            *dist_to_start/*distance_to_mm*/ = d_mm + vzd_by_degree(line->points[i].x,line->points[i].y, res_lon, res_lat);
        }
        d_mm += vzd_by_degree(line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);
   }
    return 1;
}
int imm_nearest_one(IMM_WS_DB *ws_db, IMM_WS_UNIT *ws_unit, double latitude, double longitude, IMM_MAP_PT *map_pt, IMM_ALGORITHM algorithm, IMM_MAP_SOURCE	map_source, int gps_heading)
{
	int db_index, rows_count;
	UINT best_id, best_id_noname;
	char *table;
	xdb_t*	xdb;
	xdb_result_t *xdb_result;
	char query[4096];
	const char *query_ptr = query;
	double step, mm_long, mm_lat, best_mm_long, best_mm_lat, best_dist, distance, dist_to_start,  best_mm_long_noname, best_mm_lat_noname, best_dist_noname;
	MM_WKBLINE *line;
	double degree_coef;
	int link_id = 0;
	char *street_name_native, *route_name;
	int heading;
	int headingDiff;//, bestHeadingDiff;
	int speedLimitPrev = -1, speedLimit;
	int fr_spd_lim,to_spd_lim;

	line = NULL;
	step = INITIAL_STEP2; // 0.005 = 500m
	xdb_result = NULL;
	street_name_native = NULL;
	route_name = NULL;
	rows_count = 0;

	if (((IMM_CACHE*)((ws_unit)->imm_cache))->map_source == IMM_OSM)
		table = (char *)OSM_TABLE;
	else if (((IMM_CACHE*)((ws_unit)->imm_cache))->map_source == IMM_HERE)
		table = (char *)HERE_TABLE;
	else{
		//TODO
	}

	best_dist = DBL_MAX;
	best_mm_long = DBL_MAX;
	best_mm_lat = DBL_MAX;
	best_id = 0;
	best_dist_noname = DBL_MAX;
	best_mm_long_noname = DBL_MAX;
	best_mm_lat_noname = DBL_MAX;
	best_id_noname = 0;
//	bestHeadingDiff = INT_MAX;
	BOOL unclearSpeed = FALSE;

	degree_coef = cos(latitude * (0.0174533));

	xdb = &(ws_db->xdb);

	while (rows_count == 0)
	{

		sprintf(query, "SELECT AsWKB(%s),%s,fr_spd_lim,to_spd_lim,route_name,street_name_native,%s FROM %s WHERE MBRIntersects("
				"PolygonFromText(\'POLYGON((%lf %lf,%lf %lf,%lf %lf,%lf %lf,%lf %lf))\'), geo_line)",
				NEW_LINE_ATTRIBUTE,
				(map_source == IMM_OSM) ? OSM_ID_ATTRIBUTE : HERE_ID_ATTRIBUTE,
						WAY_LENGTH_ATTRIBUTE,
						table,
						longitude - (step/degree_coef), latitude - step,
						longitude + (step/degree_coef), latitude - step,
						longitude + (step/degree_coef), latitude + step,
						longitude - (step/degree_coef), latitude + step,
						longitude - (step/degree_coef), latitude - step);

		xdb_result = xdb_query(xdb, query_ptr);

		if (xdb_result == NULL)
		{
			return 0;	//error
		}
		else
		{
			rows_count = xdb_result->row_count;
		}

		if (rows_count != 0)
		{
			while(xdb_get_next_row(xdb_result))
			{
				db_index = 0;
				street_name_native = NULL;
				route_name = NULL;

				if (xdb_get_field_blob(xdb_result, db_index++, (void**)&line) == 0){	//get one line from result
					xdb_free_result(xdb_result);
					return 0;
				}
				if (xdb_get_field_int(xdb_result, db_index++, &link_id) == 0){
					xdb_free_result(xdb_result);
					return 0;
				}

				if (xdb_get_field_int(xdb_result, db_index++, &fr_spd_lim) == 0)
					return 1;
				if (xdb_get_field_int(xdb_result, db_index++, &to_spd_lim) == 0)
					return 1;

				xdb_get_field_string(xdb_result, db_index++, &route_name);
				xdb_get_field_string(xdb_result, db_index++, &street_name_native);

				SnapToLine(line, longitude, latitude, &mm_long, &mm_lat, &distance, &dist_to_start, degree_coef, &heading);

				if(distance > 4000)
					distance = DBL_MAX;
	//			headingDiff = abs(HeadingsDifferenceAngles(heading, gps_heading));
				if(step == INITIAL_STEP2){
					headingDiff = abs(HeadingsDifferenceAngles(heading, gps_heading));
					if(headingDiff > 90)
						headingDiff = 180 - headingDiff;
					if(headingDiff > 45)
						distance = DBL_MAX;
					if(distance <(step*DEGREE_LENGTH)){
						speedLimit = (fr_spd_lim > to_spd_lim) ? fr_spd_lim : to_spd_lim;
						if(speedLimitPrev >= 0 && speedLimitPrev != speedLimit){
							unclearSpeed = TRUE;
						}
						speedLimitPrev = speedLimit;


					}
				}
				if ((best_dist > distance /*|| headingDiff < bestHeadingDiff*/) && (street_name_native!=0x0 || route_name!=0x0))
				{
	//				if((best_dist > distance && headingDiff < bestHeadingDiff) || (best_dist > distance && headingDiff<45) || (best_dist<100 && bestHeadingDiff>45)){
						best_id = link_id;
						best_dist = distance;
						best_mm_long = mm_long;
						best_mm_lat = mm_lat;
	//					bestHeadingDiff = headingDiff;
	//				}
				}
				if (best_dist_noname > distance )
				{
					best_id_noname = link_id;
					best_dist_noname = distance;
					best_mm_long_noname = mm_long;
					best_mm_lat_noname = mm_lat;
				}

				free(route_name);
				free(street_name_native);

			}

			if (best_dist != DBL_MAX || (step > 0.04 && best_dist_noname != DBL_MAX ))
			{
				if (best_dist == DBL_MAX || (best_dist > 2000 && best_dist_noname < best_dist )){

					if (algorithm != IMM_MARKOV){
						map_pt->latitude = best_mm_lat_noname;
						map_pt->longitude = best_mm_long_noname;
					}

					map_pt->id = best_id_noname;
					map_pt->diff_GPS_MM = best_dist_noname;
					if(unclearSpeed == TRUE)
						map_pt->speed_limit = -5;
					else
						map_pt->speed_limit = 5;

				}else{
					if (algorithm != IMM_MARKOV){
						map_pt->latitude = best_mm_lat;
						map_pt->longitude = best_mm_long;
					}

					map_pt->id = best_id;
					map_pt->diff_GPS_MM = best_dist;
					if(unclearSpeed == TRUE || best_dist > best_dist_noname)  //fix - set 0 speed limit if nonname road is closest one and named road selected
						map_pt->speed_limit = -5;
					else
						map_pt->speed_limit = 5;
				}

				xdb_free_result(xdb_result);
				return 1;	//OK - line found
			}else{

				xdb_free_result(xdb_result);
				rows_count = 0;

			}
		}
		else
		{
			xdb_free_result(xdb_result);
		}

		step *= STEP_RATIO;

		if (step > STEP_MAX)
		{
			map_pt->latitude = latitude;
			map_pt->longitude = longitude;
			return 0;
		}
	}

	return 0;	//no line was found
}
int imm_get_area(IMM_WS_DB *ws_db, double lat, double lon, IMM_MAP_PT *map_pt)
{

	char query[4096];
	const char *query_ptr = query;
	char *admin_area1_name_native = NULL, *admin_area1_name_transcription = NULL;
	char *admin_area3_name_native = NULL, *admin_area3_name_transcription = NULL;
	xdb_t*	xdb;
	xdb_result_t *xdb_result;

	xdb = &(ws_db->xdb);

	sprintf(query,  "SELECT admin_area1_name_native,admin_area1_name_transcription,"
			"admin_area3_name_native,admin_area3_name_transcription"
			" FROM `%s`"
			" WHERE    Within(PointFromText('POINT(%lf %lf)'), geo_line)"
			, HERE_ADMIN_TABLE, lon, lat);

	xdb_result = xdb_query(xdb, query_ptr);

	if (xdb_result == NULL)
	{
		return 0;	//error
	}

	if (xdb_result->row_count != 0)
	{
		xdb_get_next_row(xdb_result);
		xdb_get_field_string(xdb_result, 0, &admin_area1_name_native);
		xdb_get_field_string(xdb_result, 1, &admin_area1_name_transcription);
		xdb_get_field_string(xdb_result, 2, &admin_area3_name_native);
		xdb_get_field_string(xdb_result, 3, &admin_area3_name_transcription);

		if (admin_area1_name_native){
			sprintf(map_pt->country_name,"%s",admin_area1_name_native);	//COUNTRY
			free(admin_area1_name_native);
		}
		if (admin_area1_name_transcription){
			sprintf(map_pt->country_name_latin,"%s",admin_area1_name_transcription);
			free(admin_area1_name_transcription);
		}

		if (admin_area3_name_native){
			sprintf(map_pt->county_name,"%s",admin_area3_name_native);	//COUNTY
			free(admin_area3_name_native);
		}
		if (admin_area3_name_transcription){
			sprintf(map_pt->county_name_latin,"%s",admin_area3_name_transcription);
			free(admin_area3_name_transcription);
		}

	}

	xdb_free_result(xdb_result);

	return 1;	//OK
}



int imm_map_match(IMM_WS_DB *ws_db,IMM_WS_UNIT *ws_unit, IMM_REQUEST *imm_request, IMM_RESPONSE *imm_response)
{



	/* imm_init_data.map_source = IMM_HERE;
	 imm_init_data.algorithm = IMM_NEAREST;*/

	IMM_CACHE* imm_cache;
	int i;
	int result;
	IMM_GPS_PT* gps_pt;
	IMM_MAP_PT* map_pt;

	if(!imm_request->gps_pts_size) //no gps point to process
		return 1;

	imm_cache = (IMM_CACHE*)(ws_unit->imm_cache);

	gps_pt = imm_request->gps_pts;
	map_pt = imm_response->map_pts;

	if (imm_cache->map_source == IMM_HERE && imm_cache->algorithm == IMM_NEAREST)
	{
		for (i=0; i < imm_request->gps_pts_size; i++)
		{
			result = imm_nearest_one(ws_db, ws_unit, gps_pt->latitude, gps_pt->longitude, map_pt, imm_cache->algorithm, imm_cache->map_source,gps_pt->gps_heading);
			if (!result)
				imm_get_area(ws_db, gps_pt->latitude, gps_pt->longitude, map_pt);
			gps_pt++;
			map_pt++;
		}
	}

	if (imm_cache->map_source == IMM_HERE && imm_cache->algorithm != IMM_NEAREST_ONLY)
		get_additional_data(ws_db, imm_request-> gps_pts_size, imm_response->map_pts);

	return 0;


}

