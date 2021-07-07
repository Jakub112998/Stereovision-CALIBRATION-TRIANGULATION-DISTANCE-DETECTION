#include "csv.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int saveToCSV(vector<pomiar> lista, int nr)
{
    vector<int> markerIds;
    int count = 0;
    bool out;
    int n = 0;
    for (vector<pomiar>::const_iterator temp = lista.begin(); temp != lista.end(); ++temp) {
        cout << "czytamy markery\n"; 
        n++;

        markerIds.push_back(temp->markerIds);
        cout <<"temp->markerIds: "<< temp->markerIds;
        if (n == 2) { break; }
    }
    cout << "\n\npo\n\n";
    cout << "\n\n" << markerIds.size();
    csvfile csv("PomiaryZnacznik"+to_string(nr)+".csv");

    csv << "Numer znacznika" << endrow;
    for (vector<int>::const_iterator i = markerIds.begin(); i != markerIds.end(); ++i) {
        csv << to_string(*i);
        for(int z = 1; z<=3; z++)
            csv << " ";
    }
    csv << endrow;
    for (int i = 0; i < markerIds.size(); i++) {
        for (int z = 1; z <= 4; z++) {
            if (z == 1)
                csv << "X";
            else if (z == 2)
                csv << "Y";
            else if (z == 3)
                csv << "Z";
            else if (z == 4)
                csv << "Distance";
        }
    }
    csv << endrow;

    int last_moment=0, z = 1;
    for (vector<pomiar>::const_iterator temp = lista.begin(); temp != lista.end(); ++temp) {
        if (temp->markerIds == -1 && z==2) {
            csv << endrow;
            z = 1;
        }
        if(last_moment<temp->chwila)
            csv << endrow;
        for (vector<int>::const_iterator i = markerIds.begin(); i != markerIds.end(); ++i) {
            if (*i == temp->markerIds) { 
                string liczba = to_string(temp->X);
                string netX="";
                for (int z = 0; z < liczba.size(); z++)
                {
                    if (liczba[z] == '.')
                        liczba[z] = ',';
                    netX += liczba[z];
                    if (z == 8)
                        break;
                }
                liczba = to_string(temp->Y);
                string netY = "";
                for (int z = 0; z < liczba.size(); z++)
                {
                    if (liczba[z] == '.')
                        liczba[z] = ',';
                    netY += liczba[z];
                    if (z == 8)
                        break;
                }
                liczba = to_string(temp->Z);
                string netZ = "";
                for (int z = 0; z < liczba.size(); z++)
                {
                    if (liczba[z] == '.')
                        liczba[z] = ',';
                    netZ += liczba[z];
                    if (z == 8)
                        break;
                }
                liczba = to_string(temp->distance);
                string netD = "";
                for (int z = 0; z < liczba.size(); z++)
                {
                    if (liczba[z] == '.')
                        liczba[z] = ',';
                    netD += liczba[z];
                    if (z == 8)
                        break;
                }
                csv << netX << netY << netZ << netD; 
            }
        }
        last_moment = temp->chwila;
        z++;
    }
    return 0;
}