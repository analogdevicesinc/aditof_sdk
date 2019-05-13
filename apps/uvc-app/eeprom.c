/********************************************************************************/
/*  																			*/
/* @file	eeprom.c															*/
/*  																			*/
/* @brief	contains eeprom	read/write related API                          */ 
/*											*/
/*                                                                              */
/*																			    */
/* @author	Harshada Sarnaik  												*/
/*																			    */
/* @date															*/
/*  																			*/
/* 											*/
/*  																			*/
/********************************************************************************/

#include <errno.h>
#include <stdio.h>
#include <linux/fs.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include "eeprom.h"

int eeprom_open(char *dev_fqn, eeprom *e)
{
	e->fd = fopen (dev_fqn, "w+");
	if(e->fd == NULL)
	{
		fprintf(stderr, "Error eeprom_open: %s\n", strerror(errno));
		return -1;
	}
	fseek(e->fd, 0x0, SEEK_END);
	e->length = ftell(e->fd);
	return 0;
}

int eeprom_close(eeprom *e)
{
    if (e) {
        fclose(e->fd);
        e->fd = NULL;
    }
    return 0;
}

int eeprom_read_buf(eeprom *e, unsigned int addr, unsigned char *buf, int size)
{
    fprintf(stderr, "eeprom_read_buf addr: %d size: %d\n", addr, size);
	
	fseek (e->fd, addr, SEEK_SET);
    int ret = fread (buf, 1, size, e->fd);
    if (ret < size) {
        fprintf(stderr, "eeprom_read_buf failed with %d: %s\n", errno, strerror(errno));
        return -1;
    }	
    return size;
}

int eeprom_write_buf(eeprom *e, unsigned int addr, unsigned char *buf, int size)
{
    fprintf(stderr, "eeprom_write_buf addr: %d size: %d\n", addr, size);
	
	fseek (e->fd, addr, SEEK_SET);
    int ret = fwrite (buf, 1, size, e->fd);
    if (ret < size) {
        fprintf(stderr, "eeprom_write_buf failed with %d: %s\n", errno, strerror(errno));
        return -1;
    }
	
	fprintf(stderr, "eeprom_write_buf DONE\n");
    return 0;
}