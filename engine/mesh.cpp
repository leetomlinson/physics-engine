/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#include "mesh.h"


bool operator==(const Vertex &lhs, const Vertex &rhs) {
    return lhs.position == rhs.position &&
           lhs.color == rhs.color &&
           lhs.uv == rhs.uv &&
           lhs.normal == rhs.normal;
}
