struct SinglePress
{
    b4 pressed = false;
    f4 state = false;
    b4 released = true;
};

b4 single_press(SinglePress &key)
{
    if (key.released)
    {
        if (key.pressed)
        {
            key.state = true;
            key.released = false; 
        }
    }
    else
    {
        key.state = false;
        if (!key.pressed)
        {
            key.released = true;
        }
    }
    return key.state;
}

struct MYINPUT
{
    b4 quit_app;
    SinglePress w,a,s,d;
    b4 right, left, up, down;
} input;

inline void poll_events()
{
    SDL_Event e;		
    while( SDL_PollEvent( &e ) != 0 )
    {
      if( e.type == SDL_QUIT )
      {
        input.quit_app = true;
      }
      else if(e.type == SDL_KEYDOWN)
      {
        switch( e.key.keysym.sym )
        { 
            case SDLK_w: input.w.pressed = true; break;
            case SDLK_a: input.a.pressed = true; break;
            case SDLK_s: input.s.pressed = true; break;
            case SDLK_d: input.d.pressed = true; break;
            case SDLK_RIGHT: input.right = true; break;
            case SDLK_LEFT: input.left = true; break;
            case SDLK_UP: input.up = true; break;
            case SDLK_DOWN: input.down = true; break;
            case SDLK_ESCAPE: input.quit_app = true; break;
        }
      }
      else if(e.type == SDL_KEYUP)
      {
        switch( e.key.keysym.sym )
        { 
            case SDLK_w: input.w.pressed = false; break;
            case SDLK_a: input.a.pressed = false; break;
            case SDLK_s: input.s.pressed = false; break;
            case SDLK_d: input.d.pressed = false; break;
            case SDLK_RIGHT: input.right = false; break;
            case SDLK_LEFT: input.left = false; break;
            case SDLK_UP: input.up = false; break;
            case SDLK_DOWN: input.down = false; break;
            case SDLK_ESCAPE: input.quit_app = false; break;
        }
      }
    }
}