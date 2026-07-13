RANGE_N_OBJ := 2 3 4 5 6 7
RANGE_SCENES := 1 2 3 4 5
RANGE_N_CABLES := 2 3 4 5 6

.PHONY: all m_mocp-coord m_mocp-uncoord clean

all: m_mocp-coord m_mocp-uncoord

m_mocp-coord:
	@$(foreach n_o,$(RANGE_N_OBJ), \
		$(foreach s,$(RANGE_SCENES), \
			$(foreach n_c,$(RANGE_N_CABLES), \
				echo "Running: n_o=$(n_o), s=$(s), n_c=$(n_c)"; \
				m-mocp $(n_o)_obj graph_scene$(s)-m$(n_c) -c; \
			) \
		) \
	)

m_mocp-uncoord:
	@$(foreach n_o,$(RANGE_N_OBJ), \
		$(foreach s,$(RANGE_SCENES), \
			$(foreach n_c,$(RANGE_N_CABLES), \
				echo "Running: n_o=$(n_o), s=$(s), n_c=$(n_c)"; \
				m-mocp $(n_o)_obj graph_scene$(s)-m$(n_c); \
			) \
		) \
	)

clean: out
	rm -rf out/
