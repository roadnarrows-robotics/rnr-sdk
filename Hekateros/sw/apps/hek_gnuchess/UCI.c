#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


int main() {
   int sc[2];                                          ///<-Stalemate to chess pipe
   int cs[2];                                          ///<-Chess to stalemate pipe
   int rp;
   pid_t cpid;
   char *argv[8];
   char PipChk[] = {"Check\n"};
   char buf[8];
   if ( pipe(sc) == -1 ) {                             ///<-Making pipes
       printf("Failed to create pipe to Chess");
       exit(1);
   }
   if ( pipe(cs) == -1 ) {
       printf("Failed to create pipe from Chess");
       exit(1);
   }
   cpid = fork();
   if (cpid == -1) {                                  ///<-Fork fail error
       printf("could not fork to GNU Chess");
       exit(1);
   }
   if (cpid == 0) {                                   ///<-Child process
       close( sc[1]);                                 ///<-Closing unused pipe ends 
       close( cs[0]);                                 ///<-   on child end
       dup2( sc[0], 0);
       dup2( cs[1], 1);
       argv[0] = "/usr/games/gnuchess";
       argv[1] = "-x";
       argv[2] = NULL;
       write(cs[1], PipChk, (strlen(PipChk)+1));
       execvp(argv[0], argv);
       printf("exec failed");
       exit(1);
   }
   else {                                             ///<-Parent process
       printf( cpid);
       close( sc[0]);                                 ///<-Closing unused pipe ends
       close( cs[1]);                                 ///<-   on parent end
       rp = read(cs[0], buf, sizeof(buf));
       printf (buf);
       printf ("\n");
       printf("I made it");
       printf("\n"); 
       return 0;
   }
}
