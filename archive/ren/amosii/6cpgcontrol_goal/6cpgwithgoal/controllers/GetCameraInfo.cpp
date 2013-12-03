/*
 * GetCameralInfo.cpp
 *
 *  Created on: April 26, 2012
 *      Author: Ren Guanjiao
 */
 #include "GetCameraInfo.h"


GetCameraInfomation::GetCameraInfomation()
{

}

GetCameraInfomation::~GetCameraInfomation()
{

}

JNIEnv* GetCameraInfomation::create_vm(JavaVM ** jvm)
{
	JNIEnv *env;
	JavaVMInitArgs vm_args;
	JavaVMOption options;

	//Add your java class paths here
	options.optionString = "-Djava.class.path=./:/usr/share/java/jogl.jar:/usr/share/java/gluegen-rt.jar:/usr/local/share/java/lcm.jar:/home/rgj/Src/april/java/april.jar";

	//Define your JAVA version(e.g. 1.6.x)
	vm_args.version = JNI_VERSION_1_6;
	vm_args.nOptions = 1;
	vm_args.options = &options;
	vm_args.ignoreUnrecognized = 0;

    int ret = JNI_CreateJavaVM(jvm, (void**)&env, &vm_args);
    if(ret < 0)
	printf("\nFailed to Launch JVM\n");
    return env;
}

int GetCameraInfomation::GetInfo()
{
	//Launch the JVM
	JNIEnv *env;
	JavaVM * jvm;
	env = create_vm(&jvm);
	if (env == NULL)		return 1;

	//Define the class and function
	jclass clsMTT = NULL;
	jmethodID midMain = NULL;

	//Find the Class
	clsMTT = env->FindClass("TagFinder");

	jobject tagInfo = NULL;
	jclass clsR = NULL;

	if ( clsMTT != NULL )
	{
		//Find the function
		midMain = env->GetStaticMethodID( clsMTT, "TagCalculation", "()LReturnTags;");
		if (midMain != NULL)
		{
		    //Run the function
		    tagInfo = env->CallStaticObjectMethod(clsMTT, midMain, NULL);
		    clsR = env->GetObjectClass(tagInfo);
		    jint num_tags = env->GetIntField(tagInfo,env->GetFieldID(clsR,"num_tags","I"));
		    printf("C++ >>> \nFound %d Tags! \n", num_tags);

		    if(num_tags>0)
		    {
		    	//The detected tags are stored in this array "tagPos";
		    	double tagPos[num_tags][8];

		    	jfieldID arrFieldId = env->GetFieldID(clsR, "tags", "[[D");
		    	jobjectArray jobjarr = (jobjectArray) env->GetObjectField(tagInfo, arrFieldId);
		    	for(int i=0; i<num_tags; i++)
		    	{
		    		jdoubleArray jarr = (jdoubleArray)env->GetObjectArrayElement(jobjarr, i);
		    		jdouble *arr = env->GetDoubleArrayElements(jarr, 0);
		    		for(int j=0; j<8; j++)
		    		{
		    			tagPos[i][j] = arr[j];
		    		}
		    		env->ReleaseDoubleArrayElements(jarr, arr, 0);
		    		printf("C++ >>> \n Time Stamp = %f\nTag ID = %f \nXYZ = %f  %f  %f \nRPY = %f  %f  %f \n", tagPos[i][0], tagPos[i][1], tagPos[i][2], tagPos[i][3], tagPos[i][4], tagPos[i][5], tagPos[i][6], tagPos[i][7]);
		    	}
		    }
		}
		else
		{
		    printf("Unable to find the object class!\n");
		}
	}
	else
	{
		printf("Unable to find the main class!\n");
	}

    //Release the JVM
    int n = jvm->DestroyJavaVM();
    return 0;
}
