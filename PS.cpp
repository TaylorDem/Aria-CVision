#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <chrono>
#include <thread>

/*
#if defined(_WIN32) && !defined(__CYGWIN__)
#include <windows.h>
#else
#include <sys/select.h>
#endif
*/

#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>

#include "pocketsphinx.h"

#define MODELDIR "C:/Users/Taylor/Desktop/SUNYIT/Capstone/LanguageModel/"

/*
static const arg_t cont_args_def[] = {
	POCKETSPHINX_OPTIONS,
	
	{ "-argfile",
	ARG_STRING,
	NULL,
	"Argument file giving extra arguments." },
	{ "-adcdev",
	ARG_STRING,
	NULL,
	"Name of audio device to use for input." },
	{ "-infile",
	ARG_STRING,
	NULL,
	"Audio file to transcribe." },
	{ "-inmic",
	ARG_BOOLEAN,
	"no",
	"Transcribe audio from microphone." },
	{ "-time",
	ARG_BOOLEAN,
	"no",
	"Print word times in file transcription." },
	CMDLN_EMPTY_OPTION
};
*/

static ps_decoder_t *ps;
static cmd_ln_t *config;
static FILE *rawfd;

/*
* Main utterance processing loop:
*     for (;;) {
*        start utterance and wait for speech to process
*        decoding till end-of-utterance silence will be detected
*        print utterance result;
*     }
*/
static void recognize_from_microphone()
{
	ad_rec_t *ad;
	int16 adbuf[2048];
	uint8 utt_started, in_speech;
	int32 k;
	char const *hyp;
	
	if ((ad = ad_open_dev(cmd_ln_str_r(config, "-adcdev"),
		(int)cmd_ln_float32_r(config,
		"-samprate"))) == NULL)
		E_FATAL("Failed to open audio device\n");
	if (ad_start_rec(ad) < 0)
		E_FATAL("Failed to start recording\n");

	if (ps_start_utt(ps) < 0)
		E_FATAL("Failed to start utterance\n");
	utt_started = FALSE;
	printf("READY....\n");
	
	hyp = NULL;

	for (;;) {
		
		if ((k = ad_read(ad, adbuf, 2048)) < 0)
			E_FATAL("Failed to read audio\n");
		ps_process_raw(ps, adbuf, k, FALSE, FALSE);
		in_speech = ps_get_in_speech(ps);
		if (in_speech && !utt_started) {
			utt_started = TRUE;
			printf("Listening...\n");
		}
		if (!in_speech && utt_started) {
			// speech -> silence transition, time to start new utterance  
			ps_end_utt(ps);
			hyp = ps_get_hyp(ps, NULL);
			if (hyp != NULL)
				printf("%s\n", hyp);

			if (ps_start_utt(ps) < 0)
				E_FATAL("Failed to start utterance\n");
			utt_started = FALSE;
			printf("READY....\n");
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 00));
		
	}
	ad_close(ad);
}

int main1(int argc, char *argv[]) {

	
	config = cmd_ln_init(NULL, ps_args(), TRUE,
							"-jsgf", MODELDIR "Grammar.jsgf",
							"-hmm", MODELDIR "en-us",
							//"-lm", MODELDIR "en-us.lm.dmp",
							"-dict", MODELDIR "custom.dict",
							"-verbose", "yes",
							NULL);

	if (config == NULL)
		return 1;


	/*
	config = cmd_ln_parse_r(NULL, cont_args_def, argc, argv, TRUE);

	if (config && (cfg = cmd_ln_str_r(config, "-argfile")) != NULL) {
		config = cmd_ln_parse_file_r(config, cont_args_def, cfg, FALSE);
	}

	if (config == NULL || (cmd_ln_str_r(config, "-infile") == NULL && cmd_ln_boolean_r(config, "-inmic") == FALSE)) {
		E_INFO("Specify '-infile <file.wav>' to recognize from file or '-inmic yes' to recognize from microphone.");
		cmd_ln_free_r(config);
		return 1;
	}
	*/

	
	ps_default_search_args(config);
	ps = ps_init(config);

	
	if (ps == NULL) {
		printf("HELP HELP I'M BEING REPRESSED!!!\n");
		cmd_ln_free_r(config);
		return 1;
	}

	recognize_from_microphone();
	
	ps_free(ps);
	cmd_ln_free_r(config);
	

	return 0;
}
