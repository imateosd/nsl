use std.textio.all;
library nsl_simulation;

package logging is

  type log_level_t is (
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
    );

  type log_color_t is (
    LOG_COLOR_BLACK,   -- Black   30
    LOG_COLOR_RED,     -- Red 	  31
    LOG_COLOR_GREEN,   -- Green   32
    LOG_COLOR_YELLOW,  -- Yellow  33
    LOG_COLOR_BLUE,    -- Blue 	  34
    LOG_COLOR_MAGENTA, -- Magenta 35
    LOG_COLOR_CYAN,    -- Cyan 	  36
    LOG_COLOR_WHITE    -- White   37
    );                 -- Default 39
  
  function to_string(level : log_level_t) return string;

  procedure log(level : log_level_t; message : string);
  
  procedure log(level : log_level_t; message : string; color: log_color_t);

  procedure log_debug(message : string);
  procedure log_info(message : string);
  procedure log_warning(message : string);
  procedure log_error(message : string);
  procedure log_fatal(message : string);

  subtype log_context is string;

  procedure log_debug(context: log_context; message : string);
  procedure log_info(context: log_context; message : string);
  procedure log_warning(context: log_context; message : string);
  procedure log_error(context: log_context; message : string);
  procedure log_fatal(context: log_context; message : string);

end package;

package body logging is
  
  function to_string(level : log_level_t) return string is
  begin
    case level is
      when LOG_LEVEL_DEBUG   => return "DBG";
      when LOG_LEVEL_INFO    => return "INF";
      when LOG_LEVEL_WARNING => return "WRN";
      when LOG_LEVEL_ERROR   => return "ERR";
      when others            => return "FTL";
    end case;
  end function;
  
  procedure log(level : log_level_t; message : string) is
    variable l:line;
  begin
    write(l, string'("@"));
    write(l, time'image(now));
    write(l, string'(" ["));
    write(l, to_string(level));
    write(l, string'("] "));
    write(l, message);
    writeline(output, l);

    if level = LOG_LEVEL_FATAL then
      nsl_simulation.control.terminate(1);
    end if;
  end procedure;

  procedure log(level : log_level_t; message : string; color: log_color_t) is
  begin
      case color is
        when LOG_COLOR_BLACK =>
          nsl_simulation.logging.log(level => level, message => ESC & "[30m" & message & ESC & "[0m" );
        when LOG_COLOR_RED =>
          nsl_simulation.logging.log(level => level, message => ESC & "[31m" & message & ESC & "[0m" );
        when LOG_COLOR_GREEN =>
          nsl_simulation.logging.log(level => level, message => ESC & "[32m" & message & ESC & "[0m" );
        when LOG_COLOR_YELLOW =>
          nsl_simulation.logging.log(level => level, message => ESC & "[33m" & message & ESC & "[0m" );
        when LOG_COLOR_BLUE =>
          nsl_simulation.logging.log(level => level, message => ESC & "[34m" & message & ESC & "[0m" );
        when LOG_COLOR_MAGENTA =>
          nsl_simulation.logging.log(level => level, message => ESC & "[35m" & message & ESC & "[0m" );
        when LOG_COLOR_CYAN =>
          nsl_simulation.logging.log(level => level, message => ESC & "[36m" & message & ESC & "[0m" );
        when LOG_COLOR_WHITE =>
          nsl_simulation.logging.log(level => level, message => ESC & "[37m" & message & ESC & "[0m" );
      end case;
    end procedure;
  
  procedure log_debug(message : string) is
  begin
    log(LOG_LEVEL_DEBUG, message, LOG_COLOR_WHITE);
  end procedure;

  procedure log_info(message : string) is
  begin
    log(LOG_LEVEL_INFO, message, LOG_COLOR_BLUE);
  end procedure;

  procedure log_warning(message : string) is
  begin
    log(LOG_LEVEL_WARNING, message, LOG_COLOR_YELLOW);
  end procedure;

  procedure log_error(message : string) is
  begin
    log(LOG_LEVEL_ERROR, message, LOG_COLOR_RED);
  end procedure;

  procedure log_fatal(message : string) is
  begin
    log(LOG_LEVEL_FATAL, message, LOG_COLOR_RED);
  end procedure;

  procedure log_debug(context: log_context; message : string) is
  begin
    log_debug("[" & context & "] " & message);
  end procedure;

  procedure log_info(context: log_context; message : string) is
  begin
    log_info("[" & context & "] " & message);
  end procedure;

  procedure log_warning(context: log_context; message : string) is
  begin
    log_warning("[" & context & "] " & message);
  end procedure;

  procedure log_error(context: log_context; message : string) is
  begin
    log_error("[" & context & "] " & message);
  end procedure;

  procedure log_fatal(context: log_context; message : string) is
  begin
    log_fatal("[" & context & "] " & message);
  end procedure;
  
end package body;
