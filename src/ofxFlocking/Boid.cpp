/*
 *  Boid.cpp
 *  boid
 *
 *  Created by Jeffrey Crouse on 3/29/10.
 *  Copyright 2010 Eyebeam. All rights reserved.
 *  Updated by Rick Companje
 *
 */

#include "Boid.h"

Boid::Boid() {
    //loc.set(ofRandomWidth(),ofRandomHeight());
    loc.set(ofRandomWidth(),ofRandomHeight());
    vel.set(0,0);
	acc.set(0,0);
    r = 3.0;
    maxspeed = 10;
    maxforce = 0.1;
    
    /* initialize random seed: */
    //srand ( time(NULL) );
    //ofRotateZ(rand() % 180 + 1);
}

Boid::Boid(int x, int y) {
    loc.set(x,y);
    printf("loc.x: %f, loc.y: %f \n", loc.x, loc.y);
	vel.set(0,0);
	acc.set(0,0);
    r = 3.0;
    maxspeed = 10;
    maxforce = 0.1;
}

// Method to update location
void Boid::update(vector<Boid> &boids) {
	
	flock(boids);
	
    vel += acc;   // Update velocity
    vel.x = ofClamp(vel.x, -maxspeed, maxspeed);  // Limit speed
	vel.y = ofClamp(vel.y, -maxspeed, maxspeed);  // Limit speed
    loc += vel;
    //loc += acc;
    acc = 0;  // Reset accelertion to 0 each cycle
	
    //boids return from left and bottom edges
	//if (loc.x < -r) loc.x = ofGetWidth()+r;
    //if (loc.y < -r) loc.y = ofGetHeight()+r;

    //have boids return from right and top edges
    //if (loc.x > ofGetWidth()+r) loc.x = -r;
    //if (loc.y > ofGetHeight()+r) loc.y = -r;
}

void Boid::seek(ofVec2f target) {
    acc += steer(target, false);
}

void Boid::avoid(ofVec2f target) {
    acc -= steer(target, false);
}

void Boid::arrive(ofVec2f target) {
    acc += steer(target, true);
}

// A method that calculates a steering vector towards a target
// Takes a second argument, if true, it slows down as it approaches the target
ofVec2f Boid::steer(ofVec2f target, bool slowdown) {
    ofVec2f steer;  // The steering vector
    ofVec2f desired = target - loc;  // A vector pointing from the location to the target
    
	float d = ofDist(target.x, target.y, loc.x, loc.y); // Distance from the target is the magnitude of the vector
	
	
	// If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (d > 0) {
		
		desired /= d; // Normalize desired
		// Two options for desired vector magnitude (1 -- based on distance, 2 -- maxspeed)
        
		if ((slowdown) && (d < 100.0f)) {
			desired *= maxspeed * (d*100.0f); // This damping is somewhat arbitrary
		} else {
			desired *= maxspeed;
		}
		// Steering = Desired minus Velocity
		steer = desired - vel;
        //steer = desired;
		steer.x = ofClamp(steer.x, -maxforce, maxforce); // Limit to maximum steering force
		steer.y = ofClamp(steer.y, -maxforce, maxforce); 
        //if (d<15) vel.set(0,0);
		
    }
    
    return steer;
}

void Boid::draw() {
    //Draw a triangle rotated in the direction of velocity
	float angle = (float)atan2(-vel.y, vel.x);
    float theta =  -1.0*angle;
	float heading2D = ofRadToDeg(theta)+90;
    //float heading2D = ofRadToDeg(theta);

    
	ofPushStyle();
    ofFill();
    ofPushMatrix();
    ofTranslate(loc.x, loc.y);
    ofRotateZ(heading2D);
	ofBeginShape();
    ofVertex(0, -r*2);
    ofVertex(-r, r*2);
    ofVertex(r, r*2);
    ofEndShape(true);	
    ofPopMatrix();
	ofPopStyle();
}

void Boid::flock(vector<Boid> &boids) {
	ofVec2f sep = separate(boids);
	ofVec2f ali = align(boids);
	ofVec2f coh = cohesion(boids);
	
	// Arbitrarily weight these forces
	sep *= 0.5;
	ali *= 0;
	coh *= 4;
	
	acc += sep + ali + coh;
}

// Separation
// Method checks for nearby boids and steers away
ofVec2f Boid::separate(vector<Boid> &boids) {
    float desiredseparation = 25.0f;
    ofVec2f steer;
    int count = 0;
	
    // For every boid in the system, check if it's too close
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];
		
		float d = ofDist(loc.x, loc.y, other.loc.x, other.loc.y);
		
		// If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
		if ((d > 0) && (d < desiredseparation)) {
			// Calculate vector pointing away from neighbor
			ofVec2f diff = loc - other.loc;
			diff /= d;			// normalize
			diff /= d;        // Weight by distance
			steer += diff;
			count++;            // Keep track of how many
		}
    }
    // Average -- divide by how many
    if (count > 0) {
		steer /= (float)count;
    }
	
	
    // As long as the vector is greater than 0
	//float mag = sqrt(steer.x*steer.x + steer.y*steer.y);
	
	float mag = sqrt(steer.x*steer.x + steer.y*steer.y);
    if (mag > 0) {
		// Implement Reynolds: Steering = Desired - Velocity
		steer /= mag;
		steer *= maxspeed;
		steer -= vel;
		steer.x = ofClamp(steer.x, -maxforce, maxforce);
		steer.y = ofClamp(steer.y, -maxforce, maxforce);
    }
    return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
ofVec2f Boid::align(vector<Boid> &boids) {
    float neighbordist = 50.0;
    ofVec2f steer;
    int count = 0;
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];
		
		float d = ofDist(loc.x, loc.y, other.loc.x, other.loc.y);
		if ((d > 0) && (d < neighbordist)) {
			steer += (other.vel);
			count++;
		}
    }
    if (count > 0) {
		steer /= (float)count;
    }
	
    // As long as the vector is greater than 0
	float mag = sqrt(steer.x*steer.x + steer.y*steer.y);
    if (mag > 0) {
		// Implement Reynolds: Steering = Desired - Velocity
		steer /= mag;
		steer *= maxspeed;
		steer -= vel;
		steer.x = ofClamp(steer.x, -maxforce, maxforce);
		steer.y = ofClamp(steer.y, -maxforce, maxforce);
    }
    return steer;
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
ofVec2f Boid::cohesion(vector<Boid> &boids) {
    float neighbordist = 50.0;
    ofVec2f sum;   // Start with empty vector to accumulate all locations
    int count = 0;
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];
		float d = ofDist(loc.x, loc.y, other.loc.x, other.loc.y);
		if ((d > 0) && (d < neighbordist)) {
			sum += other.loc; // Add location
			count++;
		}
    }
    if (count > 0) {
		sum /= (float)count;
		return steer(sum, false);  // Steer towards the location
    }
    return sum;
}

