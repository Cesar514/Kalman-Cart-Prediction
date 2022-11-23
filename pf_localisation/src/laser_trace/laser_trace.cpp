#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <numpy/arrayobject.h>
#include <math.h>

#include <iostream>

#define MAP_VALID(i, j) ((i >= 0) && (i < map_width) && (j >= 0) && (j < map_height))
#define MAP_INDEX(i, j) ((i) + (j) * map_width)
#define GET_VAL(map_seq, i, j) (map_seq[MAP_INDEX(i,j)])


char const* greet(PyObject *x) {
    return "Hello, World!";
}

int test(boost::python::object x){
    x[0]+=10;
    return 0;
}


double map_calc_range(double ox, double oy, double oa, int map_width, int map_height,
					  double map_origin_x, double map_origin_y,
					  double map_resolution, double max_range, boost::python::object map_cells) {
  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = (floor((ox - map_origin_x) / map_resolution + 0.5) + map_width / 2);
  y0 = (floor((oy - map_origin_y) / map_resolution + 0.5) + map_height / 2);
  
  x1 = (floor(((ox + max_range * cos(oa)) - map_origin_x) / map_resolution + 0.5) + map_width / 2); 
  y1 = (floor(((oy + max_range * sin(oa)) - map_origin_y) / map_resolution + 0.5) + map_height / 2);
  
  if(abs(y1-y0) > abs(x1-x0)) {
	steep = 1;
  } else {
	steep = 0;
  }
  
  if(steep)
	{
	  tmp = x0;
	  x0 = y0;
	  y0 = tmp;
      
	  tmp = x1;
	  x1 = y1;
	  y1 = tmp;
	}
  
  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;
  
  x = x0;
  y = y0;
  
  if(x0 < x1)
	xstep = 1;
  else
	xstep = -1;
  if(y0 < y1)
	ystep = 1;
  else
	ystep = -1;
  
  if(steep)
	{
	  if(!MAP_VALID(y,x) || ( GET_VAL(map_cells,y,x) < 0 ) || (GET_VAL(map_cells,y,x) > 65))
		return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_resolution;
	}
  else
	{
	  if(!MAP_VALID(x,y) || ( GET_VAL(map_cells,x,y) < 0 ) || (GET_VAL(map_cells,x,y) > 65))
		return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_resolution;
	}
  
  while(x != (x1 + xstep * 1))
	{
	  x += xstep;
	  error += deltaerr;
	  if(2*error >= deltax)
		{
		  y += ystep;
		  error -= deltax;
		}
	  
	  if (steep)
		{
		  if(!MAP_VALID(y,x) || ( GET_VAL(map_cells,y,x) < 0 ) || (GET_VAL(map_cells,y,x) > 65) )
			return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_resolution;
		}
	  else
		{
		  if(!MAP_VALID(x,y) || ( GET_VAL(map_cells,x,y) < 0 ) || (GET_VAL(map_cells,x,y) > 65) ) 
			return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map_resolution;
		}
	}
  return max_range;
}

BOOST_PYTHON_MODULE(laser_trace)
{
    using namespace boost::python;
    def("greet", greet);
    def("test",test);
    def("map_calc_range",map_calc_range);
}
