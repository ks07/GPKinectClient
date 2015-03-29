/* Copyright 2011 Chris Kirkham, Robert Spanton

   This file is part of libkoki

   libkoki is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   libkoki is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with libkoki.  If not, see <http://www.gnu.org/licenses/>. */
#include <stdio.h>
//#include <yaml.h>
#include <assert.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "camera.h"

#include "yaml_config.h"



/**
 *
 */
static void add_param(char *key, char *value, koki_camera_params_t *params)
{

	char *endp;
	float f;


	if (strcmp(key, "frameWidth") == 0){              /* width */

		params->size.x = atoi(value);

	} else if (strcmp(key, "frameHeight") == 0){      /* height */

		params->size.y = atoi(value);

	} else if (strcmp(key, "focalLengthX") == 0){     /* focal length x */

		f = strtod(value, &endp);
		if (value != endp && *endp == '\0')
			params->focal_length.x = f;

	}  else if (strcmp(key, "focalLengthY") == 0){    /* focal length y */

		f = strtod(value, &endp);
		if (value != endp && *endp == '\0')
			params->focal_length.y = f;

	} else if (strcmp(key, "principalPointX") == 0){  /* principal point x */

		f = strtod(value, &endp);
		if (value != endp && *endp == '\0')
			params->principal_point.x = f;

	} else if (strcmp(key, "principalPointY") == 0){  /* principal point y */

		f = strtod(value, &endp);
		if (value != endp && *endp == '\0')
			params->principal_point.y = f;

	}

	/* if we get this far, just ignore it */

}



/**
 *
 */
bool koki_cam_read_params(const char *filename, koki_camera_params_t *params)
{
	// TODO: Real values, maybe from SDK?
	add_param("frameWidth", "640", params);
	add_param("frameHeight", "480", params);
	add_param("focalLengthX", "525.0", params);
	add_param("focalLengthY", "525.0", params);
	add_param("principalPointX", "320", params);
	add_param("prinicipalPointY", "240", params);
}
