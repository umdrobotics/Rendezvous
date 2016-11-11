#include "ros/ros.h" //note this must come before including 
//#include <dji_sdk/dji_drone.h>
#include <ostream>
#include <fstream>

//#include <geometry_msgs/PointStamped.h>
//#include <dji_sdk/dji_drone.h>

class DJIDrone;

class Navigation {

public: // methode

    Navigation();
    
    Navigation(DJIDrone* ptrDrone);
		
    virtual ~Navigation();
    
    void RunNavigation(void);
    
    std::ostream& GetString(std::ostream& os);

private: // members

    DJIDrone* m_ptrDrone;
	std::ofstream m_ofslog;
	DJIDrone& m_drone;
	
private: // methods
	
	void DisplayMainMenu(void);
    
	
private: // NOT IMPLEMENTED
	Navigation(const Navigation&);  // copy constructor
	Navigation& operator=(const Navigation&); // assignment

};

	
// non-member methods
std::ostream& operator<<(std::ostream& os, Navigation& navi);

