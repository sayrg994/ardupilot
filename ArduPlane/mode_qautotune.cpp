#include "mode.h"
#include "Plane.h"

bool ModeQAutotune::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQAutotune::update()
{
    plane.mode_qstabilize.update();
}

