/*
 * configuration file reader
 * reads the configuration file from the sd card
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/*
 * returns 1 if the string is a afermative answer
 */
 int afermative(char *str) {
 	if(strcmp(str, "1",1))    {return 1;}
    if(strcmp(str, "on",2))   {return 1;}
    if(strcmp(str, "On",2))   {return 1;}
	if(strcmp(str, "ON",2))   {return 1;}
	if(strcmp(str, "yes",3))  {return 1;}
    if(strcmp(str, "Yes",3))  {return 1;}
	if(strcmp(str, "YES",3))  {return 1;}
	if(strcmp(str, "true",4)) {return 1;}
	if(strcmp(str, "TRUE",3)) {return 1;}

	return 0;
 }


/*
 * returns 1 if the string is a negative answer
 */
 int negative(char *str) {
	if(strcmp(str, "0",1))     {return 1;}
	if(strcmp(str, "no",2))    {return 1;}
	if(strcmp(str, "No",2))    {return 1;}
    if(strcmp(str, "NO",2))    {return 1;}
    if(strcmp(str, "off",3))   {return 1;}
	if(strcmp(str, "Off",3))   {return 1;}
	if(strcmp(str, "OFF",3))   {return 1;}
	if(strcmp(str, "false",5)) {return 1;}
    if(strcmp(str, "false",5)) {return 1;}
	if(strcmp(str, "False",5)) {return 1;}

	return 0;
 }


/*
 * reads a boolean setting from the buffer
 * in the form of "setting"=value
 *	returns 0 for false
 *  returns 1 for true
 *	returns the default if no setting is read
 */
 int settingBoolean(char *buffer,char *setting, int default){
    char ques[100];
 	char ans[100];
 	int ret;
	sprintf(ques,"%s=\%s",setting);   //build the fromat string
	ret = sscanf(buffer, ques, &ans); //check for setting
	// if the setting was read and is afermative then return 1
	if (ret == 1 && afermative(ans)) {
		return 1;
	}
    // if the setting was read and is negative then return 0
	if (ret == 1 && negative(ans)) {
		return 0;
	}
	// else return the default
	return default;
 }

/*
 * reads a integer setting from the buffer
 * in the form of "setting"=value
 *	returns the default if no setting is read
 */
 int settingInteger(char *buffer,char *setting, int default){
 	char ques[100];
 	char ans[100];
 	int ret;
 	int val;
	sprintf(ques,"%s=\%s",setting);   //build the fromat string
	ret = sscanf(buffer, ques, &ans); //check for setting
	if(ret == 1) {
		ret = sscanf(ans, "%d", &val);    //read in the value
    	if (ret == 1)
    		return val;				 //return the value
	}
	return default;
 }

 /* 
 * reads a double setting from the buffer
 * in the form of "setting"=value
 *	returns the default if no setting is read
 */
 double settingDouble(char *buffer,char *setting, double default){
 	char ques[100];
 	char ans[100];
 	int ret;
 	double val;
	sprintf(ques,"%s=\%s",setting);   //build the fromat string
	ret = sscanf(buffer, ques, &ans); //check for setting
	if(ret == 1) {
		ret = sscanf(ans, "%lf", &val);    //read in the value
    	if (ret == 1)
    		return val;				 //return the value
	}
	return default;
 } 
