

#import "CC3PhysicsWorld.h"
#import "CC3PhysicsObject3D.h"


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

- (void) udpateGlobalTransformation:(CC3GLMatrix *)parentTransformation 
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
	[self udpateGlobalTransformation:parentTransformation]; /* Change to appropriate method */
}

- (void) setGravity:(float)x y:(float)y z:(float)z {
	_discreteDynamicsWorld->setGravity(btVector3(x, y, z));
}

- (CC3PhysicsObject3D *) createPhysicsObject:(CC3Node *)node shape:(btCollisionShape *)shape mass:(float)mass restitution:(float)restitution {
	// Create a motion state for the object
	btDefaultMotionState *motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)));
	
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
