/*
 * RobotModel.h
 *
 *  Created on: 17. jaan 2021
 *      Author: Jevgeni Kostenko
 */

#ifndef SRC_CLASSES_ROBOTMODEL_H_
#define SRC_CLASSES_ROBOTMODEL_H_

/// Main Entry point to the library
/// Init all needed properties

class RobotModel {
public:
	uint8_t ROBOT_JOINTS_COUNT = 0;
	RobotModel();
	virtual ~RobotModel();
};

#endif /* SRC_CLASSES_ROBOTMODEL_H_ */
