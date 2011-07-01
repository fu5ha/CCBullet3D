/*
 * CC3: http://isgl3d.com
 *
 * Copyright (c) 2010-2011 Stuart Caunt
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#import "CC3PhysicsWorld.h"
#import "CC3PhysicsObject3D.h"
#import "CC3MotionState.h"
#import "CC3GLU.h"

#import "btBulletDynamicsCommon.h"

@implementation CC3PhysicsWorld

- (id) init {
    if ((self = [super init])) 
	{
    	
    	_lastStepTime = [[NSDate alloc] init];
       	_physicsObjects = [[NSMutableArray alloc] init];
    }
	
    return self;
}

- (void) dealloc 
{
	
	[_lastStepTime release];
	[_physicsObjects release];

	[super dealloc];
}

- (void) setDiscreteDynamicsWorld:(btDiscreteDynamicsWorld *)discreteDynamicsWorld {
	_discreteDynamicsWorld = discreteDynamicsWorld;
}

- (void) addPhysicsObject:(CC3PhysicsObject3D *)physicsObject {
	
	// Add collision object to dynamics world
	_discreteDynamicsWorld->addRigidBody(physicsObject.rigidBody);
	
	// Add to physics list
	[_physicsObjects addObject:physicsObject];
}

- (void) removePhysicsObject:(CC3PhysicsObject3D *)physicsObject 
{
	// Remove from render list
	[physicsObject.node remove];

	// Remove collision object from dynamics world
	_discreteDynamicsWorld->removeRigidBody(physicsObject.rigidBody);
	
	// Remove from physics list
	[_physicsObjects removeObject:physicsObject];
}

- (void) removeAllChildren			
{
	[super  removeAllChildren];
	
	for (CC3PhysicsObject3D * physicsObject in _physicsObjects) {
		_discreteDynamicsWorld->removeRigidBody(physicsObject.rigidBody);
	}
	
	[_physicsObjects removeAllObjects];
}

- (void) udpateGlobalTransformation:(CC3Matrix4D *)parentTransformation 
{
	// Get time since last step
	NSDate * currentTime = [[NSDate alloc] init];
	
	NSTimeInterval timeInterval = [currentTime timeIntervalSinceDate:_lastStepTime];
//	float optimalInterval = 1. / 60.;
	
//	if (timeInterval > optimalInterval) {
//		timeInterval = optimalInterval;
//	}
	
	// Update the simulation
	_discreteDynamicsWorld->stepSimulation(timeInterval, 2);
	[_lastStepTime release];
	_lastStepTime = currentTime;

	//NSLog(@"N objects = %i", [_physicsObjects count]);

	// Update all global matrices
	[super udpateGlobalTransformation:parentTransformation]; /* Change to appropriate method */
}

- (void) setGravity:(float)x y:(float)y z:(float)z {
	_discreteDynamicsWorld->setGravity(btVector3(x, y, z));
}

- (CC3PhysicsObject3D *) createPhysicsObject:(CC3Node *)node shape:(btCollisionShape *)shape mass:(float)mass restitution:(float)restitution {
	// Create a motion state for the object
	CC3MotionState * motionState = new CC3MotionState(node);
	
	// Create a rigid body
	btVector3 localInertia(0, 0, 0);
	shape->calculateLocalInertia(mass, localInertia);
	btRigidBody * rigidBody = new btRigidBody(mass, motionState, shape, localInertia);
	rigidBody->setRestitution(restitution);
	rigidBody->setActivationState(DISABLE_DEACTIVATION);

	// Create a physics object and add it to the physics world
	CC3PhysicsObject3D * physicsObject = [[CC3PhysicsObject3D alloc] initWithNode:node andRigidBody:rigidBody];
	[self addPhysicsObject:physicsObject];
	
	return [physicsObject autorelease];
}

@end
