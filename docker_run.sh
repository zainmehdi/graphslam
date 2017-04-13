#!/bin/sh
docker run -v $PWD:/home/${PWD##*/} -it ${PWD##*/}
