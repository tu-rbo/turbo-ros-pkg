/* * IAP - Interactive Perception System - Segment visually observable environment
 * into rigid bodies and estimate type and properties of joints between them by
 * means of interaction.
 * Copyright (C) 2012 Technische Universitaet Berlin - RBO
 * <robotics@robotics.tu-berlin.de>
 * 
 * This file is part of IAP.
 * 
 * IAP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * IAP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with IAP.  If not, see <http://www.gnu.org/licenses/>.
 */
/* 
 *  Copyright (c) 2008-2010  Noah Snavely (snavely (at) cs.cornell.edu)
 *    and the University of Washington
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/* CreateMatchScript.cpp */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

int mainrano(int argc, char **argv)
{
    if (argc < 2 || argc > 5) {
        printf("Usage: %s <list.txt> [key_dir] [match_dir]\n", argv[0]);
        return 1;
    }
    
    char *list_in = argv[1];
    
    char *key_dir = NULL;
    char *match_dir = NULL;
    
    if (argc > 2)
        key_dir = argv[2];
    
    if (argc > 3)
        match_dir = argv[3];

    /* Read the list */
    std::vector<std::string> key_files;
    
    FILE *f = fopen(list_in, "r");
    
    if (f == NULL) {
        printf("Error opening file %s for reading\n", list_in);
        return 1;
    }

    char buf[256];

    while (fgets(buf, 256, f)) {
        /* Remove trailing newline */
        if (buf[strlen(buf) - 1] == '\n')
            buf[strlen(buf) - 1] = 0;

        buf[strlen(buf) - 3] = 'k';
        buf[strlen(buf) - 2] = 'e';
        buf[strlen(buf) - 1] = 'y';

        key_files.push_back(std::string(buf));    
    }
    
    int num_files = (int) key_files.size();

    for (int i = 0; i < num_files; i++) {
        for (int j = i+1; j < num_files; j++) {
            if (key_dir && !match_dir) {
                printf("KeyMatch %s/%s %s/%s match-%03d-%03d.txt\n",
                       key_dir, key_files[i].c_str(), 
                       key_dir, key_files[j].c_str(), i, j);
            } else if (key_dir && match_dir) {
                printf("KeyMatch %s/%s %s/%s %s/match-%03d-%03d.txt\n",
                       key_dir, key_files[i].c_str(), 
                       key_dir, key_files[j].c_str(), 
                       match_dir, i, j);
            } else {
                printf("KeyMatch %s %s match-%03d-%03d.txt\n",
                       key_files[i].c_str(), key_files[j].c_str(), i, j);
            }
        }
    }

    return 0;
}
