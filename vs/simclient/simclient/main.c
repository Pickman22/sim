int RunAllTests(void);
int app(void);

int main(void) {
#ifdef RUN_TESTS
	RunAllTests();
#else
	app();
#endif
}