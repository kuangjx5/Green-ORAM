#include <stdio.h>
#include "utlist.h"
#include "utils.h"

#include "memory_controller.h"

extern long long int CYCLE_VAL;

extern double baseReadNum;
extern double pathReadNum;
extern double d1_pathReadNum;
extern double d2_pathReadNum;
extern double d3_pathReadNum;
extern double PA_ReadNum;
extern double path_PA_ReadNum;
extern double PAPR_ReadNum;
extern double PAPRA_ReadNum;
extern double path_PAPR_ReadNum;
extern double path_PAPRA_ReadNum;

extern double basePower;//path_ORAM
extern double pathPower;//fork;+multi
extern double PA_Power;//PA;+multi
extern double path_PA_Power;//PA+fork;+multi

extern double PAPR_Power;
extern double PAPRA_Power;
extern double path_PAPR_Power;
extern double path_PAPRA_Power;

#define REF_POWER 187.5

long long int totalcnt;

void init_scheduler_vars()
{
	// initialize all scheduler variables here

	return;
}

// write queue high water mark; begin draining writes if write queue exceeds this value
#define HI_WM 40

// end write queue drain once write queue has this many writes in it
#define LO_WM 20

// 1 means we are in write-drain mode for that channel
int drain_writes[MAX_NUM_CHANNELS];

/* Each cycle it is possible to issue a valid command from the read or write queues
   OR
   a valid precharge command to any bank (issue_precharge_command())
   OR 
   a valid precharge_all bank command to a rank (issue_all_bank_precharge_command())
   OR
   a power_down command (issue_powerdown_command()), programmed either for fast or slow exit mode
   OR
   a refresh command (issue_refresh_command())
   OR
   a power_up command (issue_powerup_command())
   OR
   an activate to a specific row (issue_activate_command()).

   If a COL-RD or COL-WR is picked for issue, the scheduler also has the
   option to issue an auto-precharge in this cycle (issue_autoprecharge()).

   Before issuing a command it is important to check if it is issuable. For the RD/WR queue resident commands, checking the "command_issuable" flag is necessary. To check if the other commands (mentioned above) can be issued, it is important to check one of the following functions: is_precharge_allowed, is_all_bank_precharge_allowed, is_powerdown_fast_allowed, is_powerdown_slow_allowed, is_powerup_allowed, is_refresh_allowed, is_autoprecharge_allowed, is_activate_allowed.
   */

void schedule(int channel)
{
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;
	totalcnt++;

	// if in write drain mode, keep draining writes until the
	// write queue occupancy drops to LO_WM
	if (drain_writes[channel] && (write_queue_length[channel] > LO_WM)) {
	  drain_writes[channel] = 1; // Keep draining.
	}
	else {
	  drain_writes[channel] = 0; // No need to drain.
	}

	// initiate write drain if either the write queue occupancy
	// has reached the HI_WM , OR, if there are no pending read
	// requests
	if(write_queue_length[channel] > HI_WM)
	{
		drain_writes[channel] = 1;
	}
	else {
	  if (!read_queue_length[channel])
	    drain_writes[channel] = 1;
	}


	// If in write drain mode, look through all the write queue
	// elements (already arranged in the order of arrival), and
	// issue the command for the first request that is ready
	if(drain_writes[channel])
	{

		LL_FOREACH(write_queue_head[channel], wr_ptr)
		{
			if(wr_ptr->command_issuable)
			{
				issue_request_command(wr_ptr);
				break;
			}
		}
		return;
	}

	// Draining Reads
	// look through the queue and find the first request whose
	// command can be issued in this cycle and issue it 
	// Simple FCFS 
	if(!drain_writes[channel])
	{
		LL_FOREACH(read_queue_head[channel],rd_ptr)
		{
			if(rd_ptr->command_issuable)
			{
				issue_request_command(rd_ptr);
				break;
			}
		}
		return;
	}
}

void scheduler_stats()
{
  /* Nothing to print for now. */
	printf("totalcnt\t%lld\n", totalcnt);
	printf("baseReadNum\t%lf\n", baseReadNum);

	printf("pathReadNum\t%lf\n", pathReadNum);
	printf("d1_pathReadNum\t%lf\n", d1_pathReadNum);
	printf("d2_pathReadNum\t%lf\n", d2_pathReadNum);
	printf("d3_pathReadNum\t%lf\n", d3_pathReadNum);

	printf("PA_ReadNum\t%lf\n", PA_ReadNum);
	printf("path_PA_ReadNum\t%lf\n", path_PA_ReadNum);

	printf("PAPR_ReadNum\t%lf\n", PAPR_ReadNum);
	printf("PAPRA_ReadNum\t%lf\n", PAPRA_ReadNum);
	printf("path_PAPR_ReadNum\t%lf\n", path_PAPR_ReadNum);
	printf("path_PAPRA_ReadNum\t%lf\n", path_PAPRA_ReadNum);

	printf("basePower+ref\t%lf\n", basePower+baseReadNum*REF_POWER);
	printf("pathPower+ref\t%lf\n", pathPower+baseReadNum*REF_POWER);
	printf("PA_Power+ref\t%lf\n", PA_Power+baseReadNum*REF_POWER);
	printf("path_PA_Power+ref\t%lf\n", path_PA_Power+baseReadNum*REF_POWER);
	printf("PAPR_Power+ref\t%lf\n", PAPR_Power+baseReadNum*REF_POWER);
	printf("PAPRA_Power+ref\t%lf\n", PAPRA_Power+baseReadNum*REF_POWER);
	printf("path_PAPR_Power+ref\t%lf\n", path_PAPR_Power+baseReadNum*REF_POWER);
	printf("path_PAPRA_Power+ref\t%lf\n", path_PAPRA_Power+baseReadNum*REF_POWER);
}

