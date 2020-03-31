# -*- coding: utf-8 -*-
# Copyright 2016, Olivier Stasse, CNRS

from dynamic_graph.sot.tiago import Tiago


class TiagoSteel(Tiago):
    """
    This class instantiates LAAS TIAGO Robot
    """
    def defineHalfSitting(self, q):
        Tiago.defineHalfSitting(self, q)

    def __init__(self, name, device=None, tracer=None, with_wheels=True,
            fromRosParam=False):
        Tiago.__init__(self, name, device=device, tracer=tracer,
                with_wheels=with_wheels,
                fromRosParam=fromRosParam)

__all__ = ["TiagoSteel"]
