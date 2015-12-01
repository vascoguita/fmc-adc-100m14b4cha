#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#define FALD_TRG_ARGC 2

static char git_version[] = "version: " GIT_VERSION;
static char zio_git_version[] = "zio version: " ZIO_GIT_VERSION;

void send_config(int fd, char *msg)
{
	/* removing newline at the end */
	if (msg[strlen(msg)-1] == '\n')
		msg[strlen(msg)-1] = '\0';

	if (strlen(msg)) {
		write(fd, msg, strlen(msg));
		fprintf(stdout, "New trig setting sent: %s(len: %d)\n",
			msg, (int)strlen(msg));
	} else {
		fprintf(stdout, "Nothing sent due to an empty user input\n");
	}
}

static void print_version(char *pname)
{
	printf("%s %s\n", pname, git_version);
	printf("%s %s\n", pname, zio_git_version);
}

int main(int argc, char *argv[])
{
	int i, fd;
	char adcfifo[128];
	char msg[512], *ptr;
	unsigned int devid;

	if ((argc >= 2) && (!strcmp(argv[1], "-V"))) {
		print_version(argv[0]);
		exit(0);
	}

	if (argc < FALD_TRG_ARGC) {
		fprintf(stderr, "\nUsage:\nfald-trg-cfg [-V] <dev_id> [options]\n");
		return -1;
	}

	sscanf(argv[FALD_TRG_ARGC - 1], "%x", &devid);
	if (access(adcfifo, F_OK) == -1) {
		sprintf(adcfifo, "/tmp/adcfifo-%04x", devid);
		/* create the FIFO (named pipe) */
		mkfifo(adcfifo, 0666);
	}

	fd = open(adcfifo, O_WRONLY);
	if (fd == -1) {
		fprintf(stdout, "open %s failed errno:%d\n", adcfifo, errno);
		exit(1);
	}

	/*
	 * If we have parameters from the command line, then send them
	 * immediately and close the program. This allow external program to
	 * invoke this one for the configuration instead of interactive input
	 */
	if (argc > FALD_TRG_ARGC) {
		memset(msg, 0, 512);
		for (i = 2; i < argc; ++i) {
			strcat(msg, argv[i]);
			strcat(msg, " ");
		}
		send_config(fd, msg);
		close(fd);

		return 0;
	}

	/* get user input */
	while (1) {
		memset(msg, 0, 512);
		fprintf(stdout, "Change trig config using standard args: -a -b -c -n -e\n >>>:  ");
		ptr = fgets(msg, sizeof(msg), stdin);
		if (!ptr) {
			fprintf(stderr, "Error while reading options\n");
			break;
		}

		send_config(fd, msg);
	}
	close(fd);
	/* don't remove the FIFO  to not break the reader side */
	//unlink(adcfifo);
	return 0;
}

