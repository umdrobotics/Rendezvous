#include "ros/ros.h" //note this must come before including 
//#include <dji_sdk/dji_drone.h>
#include <ostream>
#include <fstream>

//#include <geometry_msgs/PointStamped.h>
//#include <dji_sdk/dji_drone.h>

class DJIDrone;

class Navigation {

public: // methods

    // default constructor
    Navigation();
    
    // constructor
    Navigation(ros::NodeHandle& nh);
		
	// destructor	
    virtual ~Navigation();
    
    // 
    void RunNavigation(void);
    
    
    std::ostream& GetString(std::ostream& os);

private: // members
	
    DJIDrone* m_ptrDrone;
	std::ofstream m_ofslog;
	
	
private: // methods
	
	void DisplayMainMenu(void);
    void DrawCircleExample(void);
    void SearchForTarget(void);
	
private: // NOT IMPLEMENTED
	Navigation(const Navigation&);  // copy constructor
	Navigation& operator=(const Navigation&); // assignment

};

	
// non-member methods
std::ostream& operator<<(std::ostream& os, Navigation& navi);

