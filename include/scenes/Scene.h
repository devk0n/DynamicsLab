#ifndef SCENE_H
#define SCENE_H

struct Context;

class Scene {
public:
  explicit Scene(const Context &ctx) : m_ctx(ctx) {}
  virtual ~Scene() = default;

  virtual bool load() = 0;
  virtual void update(double dt) = 0;
  virtual void render() = 0;
  virtual void unload() = 0;

protected:
  const Context &m_ctx;
};

#endif // SCENE_H
