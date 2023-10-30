#pragma once

struct Point2d
{
    float x;
    float y;

    // return an array of the x and y values in json format
    String toJsonString() {
        String jsonString = "[";
        jsonString += x;
        jsonString += ",";
        jsonString += y;
        jsonString += "]";
        return jsonString;
    }
};
