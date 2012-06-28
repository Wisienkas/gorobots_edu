/*
 * GetCameralInfo.h
 *
 *  Created on: April 26, 2012
 *      Author: Ren Guanjiao
 */
#ifndef GETCAMERAINFO_H_
#define GETCAMERAINFO_H_

#include <stdio.h>
#include <jni.h>
#include <string.h>

//Getting the information from camera
Class GetCameraInfomation
{
public:
	GetCameraInfomation();
	~GetCameraInfomation();

	//functions
	JNIEnv* create_vm(JavaVM ** jvm);
	int GetInfo();

};


#endif
