#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;
using namespace std;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    float start_x, start_y, end_x, end_y;
    bool start_x_is_valid(false);
    bool start_y_is_valid(false);
    bool end_x_is_valid(false);
    bool end_y_is_valid(false);

    while ( not start_x_is_valid || not start_y_is_valid || not end_x_is_valid || not end_y_is_valid )
    {
    	if ( not start_x_is_valid )
    	{
			cout << "Please enter the x coordinate for the starting point : ";
			if ( std::cin >> start_x )
			{
				start_x_is_valid = true;
				//cout << "The staring x coordinate is valid ";
			}
			else
			{
				cout << " Wrong input, ["<<start_x<<"] is not a float \n";
				std::cin.clear(); // reset state
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // consume wrong input
			}
    	}
    	if ( start_x_is_valid && not start_y_is_valid )
    	{
    		cout << "Please enter the y coordinate for the starting point : ";
			if ( std::cin >> start_y )
			{
				start_y_is_valid = true;
				//cout << "The staring y coordinate is valid ";
			}
			else
			{
				cout << " Wrong input, ["<<start_y<<"] is not a float \n";
				std::cin.clear(); // reset state
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // consume wrong input
			}
    	}
    	if ( start_y_is_valid && not end_x_is_valid )
    	{
			cout << "Please enter the x coordinate for the ending point : ";
			if ( std::cin >> end_x )
			{
				end_x_is_valid = true;
				//cout << "The ending x coordinate is valid ";
			}
			else
			{
				cout << " Wrong input, ["<<end_x<<"] is not a float \n";
				std::cin.clear(); // reset state
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // consume wrong input
			}
    	}
    	if ( end_x_is_valid && not end_y_is_valid )
    	{
			cout << "Please enter the y coordinate for the ending point : ";
			if ( std::cin >> end_y )
			{
				end_y_is_valid = true;
				//cout << "The ending y coordinate is valid ";
			}
			else
			{
				cout << " Wrong input, ["<<end_y<<"] is not a float \n";
				std::cin.clear(); // reset state
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // consume wrong input
			}
    	}

    }

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
