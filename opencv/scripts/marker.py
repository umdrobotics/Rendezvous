#!/usr/bin/env python2
# -*- coding: UTF-8 -*-


"""
This class is to record marker infomation: id and pixel points.
"""
class marker:

    def __init__(self):
        self.markerId = -1
        # self.markerPoints = []


    def saveMarker(self, inMarker, inMarkerId):
        self.markerId = inMarkerId
        self.markerPoints = inMarker
