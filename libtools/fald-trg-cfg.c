#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

int main()
{
	int fd;
	char *adcfifo = "/tmp/adcfifo";
	char msg[512], *ptr;

	if (access(adcfifo, F_OK) == -1) {
		/* create the FIFO (named pipe) */
		mkfifo(adcfifo, 0666);
	}

	fd = open(adcfifo, O_WRONLY);
	if (fd == -1) {
		fprintf(stdout, "open %s failed errno:%d\n", adcfifo, errno);
		exit(1);
	}
	/* get user input */
	for (;;) {
		memset(msg, 0, 512);
		fprintf(stdout, "Change trig config using standard args: -a -b -c -n -e\n >>>:  ");
		ptr = fgets(msg, sizeof(msg), stdin);
		if (!ptr) {
			fprintf(stderr, "Error while reading options\n");
			break;
		}
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
		continue;
	}
	close(fd);
	/* don't remove the FIFO  to not break the reader side */
	//unlink(adcfifo);
	return 0;
}

