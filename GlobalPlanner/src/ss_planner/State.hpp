//
//  State.h
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__State__
#define __GlobalPlanner__State__

#include <cmath>
#include <string>
#include <iostream>

namespace navigation    {
    
    class State {
        
        // TODO: find a way make it immutable
        // load in swapping after making these const
        int xCordinate_ = 0, yCordinate_ = 0;
        double theta_ = 0, curvature_ = 0;
        
        static const double inRange(const double theta_) ;
        
    public:
        inline int x()         const       { return xCordinate_;   }
        inline int y()         const       { return yCordinate_;   }
        inline double theta()     const       { return theta_;        }
        inline double curvature() const       { return curvature_;    }
        
        //TODO : removue this
        inline State(){    }
        inline State(int xCordinate, int yCordinate, double theta, double curvature ) : xCordinate_{xCordinate}, yCordinate_{yCordinate}, theta_{inRange(theta)}, curvature_{curvature}    {}
        
        inline double distanceSqTo(const State& b) const    {
            
            return (xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_);
            
        }
        
        inline double distanceTo(const State& b) const {
            
            return sqrt(xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_);
        
            
        }
        
        inline double euclidianDistanceTo(const State& b) const {
            
            return sqrt(xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_);
            
            
        }
        
        inline double manhattanDistanceTo(const State& b) const {
            
            return abs(xCordinate_ - b.xCordinate_) + abs(yCordinate_ - b.yCordinate_);
            
            
        }
        
        inline bool operator==(const State& b) const    {
            return xCordinate_==b.x() && yCordinate_==b.y() && theta_==b.theta() && curvature_==curvature();
        }
    	
		inline bool operator!=(const State& b) const    {
            return !(xCordinate_==b.x() && yCordinate_==b.y() && theta_==b.theta() && curvature_==curvature());
        }
        
        const std::string toString() const;
        
//        const std::ostream& operator<<(const std::ostream& os, const State& state);
        
    };
    
    typedef std::shared_ptr<State> StatePtr;
}


#endif /* defined(__GlobalPlanner__State__) */
