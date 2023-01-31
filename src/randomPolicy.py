#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from dotenv import dotenv_values 

def randomPolicy(state,time):
        rng=np.random.default_rng()
        maxPressure=0.01

        p0= np.random.default_rng().uniform(0,maxPressure)
        print("pressure = "+ p0.__str__())

        return p0
