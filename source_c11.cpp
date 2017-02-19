#include <vector>
#include <array>

#include "plus/plus.h"

#include "scene/components/camera.h"
#include "scene/components/light.h"
#include "scene/components/simple_graphic_scene_overlay.h"
#include "scene/components/transform.h"
#include "scene/scene_picking.h"
#include "scene/systems/renderable_system.h"
#include "scene_serialization/scene_serialization.h"
#include "scene_serialization/scene_serialization_context.h"
#include "serialization/type_serialization.h"

#include "filesystem/filesystem.h"
#include "filesystem/path_tools.h"
#include "iso_surface/iso_surface.h"
#include "picture/picture_io.h"

#include "imgui.h"
#include "init/init.h"

#include "io_core_drivers/io_cfile.h"
#include "io_zip/io_zip.h"

using namespace gs;

/* PARTICLE FIELD */

ByteArray heightmap;

Vector3 world_to_field(const Vector3 &w);

Vector3 field_min(-16, 0, -16), field_max(16, 4, 16), field_res(1, 1, 1), field_size = field_max - field_min;

static const float cohesion_limit = 2.f;
static const float field_collision_restitution = 0.5f;

struct particle {
	Vector3 pos, vel, acc;
};

void init_particle(particle &p, const Vector3 &pos) {
	p.pos = pos;
	p.vel.Set(0, 0, 0);
	p.acc.Set(0, 0, 0);
}

std::vector<particle> particles;

//
struct totem {
	Vector3 pos;
};

std::array<totem, 3> totems;
uint active_totems = 0;

struct home {
	Vector3 pos;
	float energy;
};

std::vector<home> homes;
float total_homes_energy;
bool take_damage = false;

//
void reset_homes_energy() {
	for (auto &h : homes)
		h.energy = 10.f;
}

float get_homes_energy() {
	float energy = 0.f;
	for (auto &h : homes)
		energy += h.energy;
	return energy;
}

void spawn_homes(core::Scene &scn) {
	const auto nodes = scn.GetNodes();

	uint home_count = 0;
	for (uint i = 0; i < nodes.size(); ++i)
		if (starts_with(nodes[i]->GetName(), "maison"))
			++home_count;

	log(stringify("home count: %1").arg(home_count));
	homes.resize(home_count);

	home_count = 0;
	for (uint i = 0; i < nodes.size(); ++i)
		if (starts_with(nodes[i]->GetName(), "maison")) {
			homes[home_count].pos = nodes[i]->GetComponent<core::Transform>()->GetWorld().GetTranslation();
			++home_count;
		}

	reset_homes_energy();
	total_homes_energy = get_homes_energy();
}

//
void create_particle_field() {
	auto field_size = field_max - field_min;
	field_size /= field_res;

	int particle_count = field_size.x * field_size.y * field_size.z;

	particles.resize(particle_count);

	int i = 0;
	for (auto x = field_min.x; x < field_max.x; x += field_res.x) {
		for (auto y = field_min.y; y < field_max.y; y += field_res.y) {
			for (auto z = field_min.z; z < field_max.z; z += field_res.z) {
				init_particle(particles[i], Vector3(x, y, z));
				++i;
			}
		}
	}

	__ASSERT__(i == particle_count);

	log(stringify("%1 particle(s)").arg(particle_count));
}

// 7.f
const float altitude_min = 5.66898f, altitude_max = 47.22528f;

void particle_sample_ground(const Vector3 &pos, Vector3 &n, float &h) {
	auto p = (pos - field_min) / field_size;
	p.z = 1.f - p.z;
	int u = p.x * 1024.f, v = p.z * 1024.f;

	u = types::Clamp(u, 0, 1013);
	v = types::Clamp(v, 0, 1013);

	float *p_f = reinterpret_cast<float *>(heightmap.data());

	float hc = p_f[u + v * 1024];
	float hr = p_f[(u + 10) + v * 1024];
	float hb = p_f[u + (v + 10) * 1024];

	Vector3 i(0.1, hr - hc, 0), j(0, hb - hc, -0.1);
	n = i.Normalized().Cross(j.Normalized());

	h = hc * altitude_max + altitude_min;
}

#define AXIS_ACCEL x

void update_particle_field() {
	auto count = particles.size();

	// SAP?
	std::sort(particles.begin(), particles.end(), [](const particle &a, const particle &b) { return a.pos.AXIS_ACCEL < b.pos.AXIS_ACCEL; });

	// cohesion/repulsion
	int nn_count = 0;

#if 1
	for (int i = 0; i < count; ++i) {
		auto &p_a = particles[i];

		// determine range start
		int j = i;
		for (; j > 0; --j)
			if (particles[j].pos.AXIS_ACCEL < p_a.pos.AXIS_ACCEL - cohesion_limit)
				break; // too far behind

		for (; j < count; ++j) {
			if (i == j)
				continue;

			++nn_count;

			auto &p_b = particles[j];

			if (p_b.pos.AXIS_ACCEL > p_a.pos.AXIS_ACCEL + cohesion_limit)
				break; // too far front

			auto a_to_b = p_b.pos - p_a.pos;
			auto a_to_b_len = a_to_b.Len();

			if (!a_to_b_len)
				continue;

			if (a_to_b_len > cohesion_limit)
				continue;

			float k;
			if (a_to_b_len > 1.f) {
				k = (cohesion_limit - a_to_b_len) * -0.001f;
			}
			else {
				k = (1.f - a_to_b_len) * 0.475f;
			}

			k = k * k;

			auto I = a_to_b * k;
			p_a.acc -= I;
			p_b.acc += I;
		}
	}
#else
	for (int i = 0; i < count; ++i)
		for (int j = 0; j < count; ++j) {
			if (i == j)
				continue;

			++nn_count;

			auto &p_a = particles[i];
			auto &p_b = particles[j];

			auto a_to_b = p_b.pos - p_a.pos;
			auto a_to_b_len = a_to_b.Len();

			if (!a_to_b_len)
				continue;

			if (a_to_b_len > cohesion_limit)
				continue;

			float k;
			if (a_to_b_len > 1.f) {
				k = (cohesion_limit - a_to_b_len) * -0.001f;
			}
			else {
				k = (1.f - a_to_b_len) * 0.475f;
			}

			k = k * k;

			auto I = a_to_b * k;
			p_a.acc -= I;
			p_b.acc += I;
		}
#endif

	//	log(stringify("pair tested: %1").arg(nn_count));

	// totem repulsion
	static const float totem_repulsion_dist = 2.0f;

	for (uint i = 0; i < active_totems; ++i) {
		Vector3 totem_field_pos = world_to_field(totems[i].pos);

		for (int j = 0; j < count; ++j) {
			auto &p = particles[j];

			auto p_to_totem = p.pos - totem_field_pos;
			p_to_totem.y = 0.f; // cylinder
			auto d_to_totem = p_to_totem.Len();

			if (d_to_totem > totem_repulsion_dist)
				continue;

			float k = totem_repulsion_dist - d_to_totem;
			auto repulsion = p_to_totem * (k / d_to_totem);

			p.acc += repulsion * 1.f;
		}
	}

	// home damage
	if (take_damage)
		for (uint i = 0; i < homes.size(); ++i) {
			Vector3 home_field_pos = world_to_field(homes[i].pos);

			for (int j = 0; j < count; ++j) {
				auto &p = particles[j];

				auto p_to_totem = p.pos - home_field_pos;
				auto d_to_totem = p_to_totem.Len();

				if (d_to_totem > 1.f)
					continue;

				float damage = p.vel.Len();
				homes[i].energy -= damage * 0.6f;
			}
		}

	// constraint & integration
	for (int i = 0; i < count; ++i) {
		auto &p = particles[i];

		// gravity
		p.acc.y -= 0.025f;

		// field limit constraints
		if (p.pos.x > field_max.x) {
			p.pos.x = field_max.x;
			p.vel.x *= -field_collision_restitution;
		}
		if (p.pos.z > field_max.z) {
			p.pos.z = field_max.z;
			p.vel.z *= -field_collision_restitution;
		}
		if (p.pos.x < field_min.x) {
			p.pos.x = field_min.x;
			p.vel.x *= -field_collision_restitution;
		}
		if (p.pos.z < field_min.z) {
			p.pos.z = field_min.z;
			p.vel.z *= -field_collision_restitution;
		}

		// integration
		p.vel += p.acc;
		p.pos += p.vel;
		p.acc.Set(0, 0, 0);

		// floor
		Vector3 n;
		float y_ground;
		particle_sample_ground(p.pos, n, y_ground);
		y_ground /= 4; // field is 4 unit high, iso is 16 unit high

		if (p.pos.y < y_ground) {
			float d = y_ground - p.pos.y;
			p.vel.y = 0.f; // stop current motion
			p.vel += n * d * 0.1f;
		}

		// damping
		p.vel *= 0.98f;
	}
}

void draw_cross(core::SimpleGraphicSceneOverlay &gfx, const Vector3 &pos) {
	//	gfx.Line(pos.x - 0.1, pos.y, pos.z, pos.x + 0.1, pos.y, pos.z, Color::White, Color::White);
	//	gfx.Line(pos.x, pos.y - 0.1, pos.z, pos.x, pos.y + 0.1, pos.z, Color::White, Color::White);
	gfx.Line(pos.x, pos.y, pos.z - 0.1, pos.x, pos.y, pos.z + 0.1, Color::White, Color::White);
}

void apply_wave(float k = 0.01f) {
	auto count = particles.size();
	for (int i = 0; i < count; ++i) {
		auto &p = particles[i];
		p.vel.z += (field_max.z - p.pos.z) * k;
	}
}

//
static const int iso_scale = 2;

int iso_w = 212 / iso_scale, iso_h = 64 / iso_scale, iso_d = 212 / iso_scale;
Vector3 iso_min(-212 / iso_scale, 0, -212 / iso_scale), iso_max(212 / iso_scale, 64 / iso_scale, 212 / iso_scale), iso_unit(iso_scale, iso_scale, iso_scale);

core::ScenePicking *scene_picking;

auto water_iso = std::make_shared<core::IsoSurface>(); // x -> z -> y
std::vector<float> water_field;

render::sGeometry water_geo;
render::sMaterial water_mat;

const auto particle_to_iso_cell = Vector3(iso_w, iso_h, iso_d) / (Vector3(field_max.x, 16, field_max.z) - field_min);

Vector3 world_to_field(const Vector3 &w) {
	return (w - iso_min) * (field_max - field_min) / (iso_max - iso_min) + field_min;
}

void init_water() {
	water_geo = std::make_shared<render::Geometry>();
	water_mat = g_plus->GetRenderSystem()->LoadMaterial("water.mat");

	water_field.resize(iso_w * iso_d * iso_h);
}

void particles_to_iso_field() {
	auto count = particles.size();

	std::fill(water_field.begin(), water_field.end(), 0);

	static const int particle_width = 4;

	for (uint i = 0; i < count; ++i) {
		auto &p = particles[i];

		// transform from particle space to iso cell space
		auto cell_p = (p.pos - field_min) * particle_to_iso_cell;

		// compute particle cell
		int cell_x = cell_p.x, cell_y = cell_p.y, cell_z = cell_p.z;

		for (int c_x = cell_x - particle_width; c_x <= cell_x + particle_width; ++c_x) {
			for (int c_z = cell_z - particle_width; c_z <= cell_z + particle_width; ++c_z) {
				for (int c_y = cell_y - particle_width; c_y <= cell_y + particle_width; ++c_y) {
					if ((c_x >= 0 && c_y >= 0 && c_z >= 0) && (c_x < iso_w && c_y < iso_h && c_z < iso_d)) {
						int c_i = c_x + c_z * iso_w + c_y * iso_w * iso_d;
						float d = Vector3::Dist(Vector3(c_x, c_y, c_z), cell_p);
						float v = math::Max(4.f - d, 0.f) / 4.f;

						v = (v * v * v);
						water_field[c_i] += v * 2.f;
					}
				}
			}
		}
	}

#if 0
	// field blur
	uint h_offset = iso_w * iso_d;

	// X pass
	for (uint y = 0; y < iso_h; ++y) {
		float *p_data = water_field.c_ptr() + (h_offset * y) + 1;

		for (uint z = 0; z < iso_d; ++z) {
			for (uint x = 1; x < iso_w - 1; ++x)
				p_data[x] = (p_data[x - 1] + p_data[x] * 2 + p_data[x + 1]) / 4;
			p_data += iso_w;
		}
	}

	// Y pass
	for (uint z = 0; z < iso_d; ++z) {
		float *p_data = water_field.c_ptr() + (iso_w * z) + h_offset;

		for (uint y = 1; y < iso_h - 1; ++y) {
			for (uint x = 0; x < iso_w; ++x)
				p_data[x] = (p_data[int(x) - int(h_offset)] + p_data[x] * 2 + p_data[x + h_offset]) / 4;
			p_data += h_offset;
		}
	}

	// Z pass
	for (uint y = 0; y < iso_h; ++y) {
		float *p_data = water_field.c_ptr() + (h_offset * y) + iso_w;

		for (uint z = 1; z < iso_d - 1; ++z) {
			for (uint x = 0; x < iso_w; ++x)
				p_data[x] = (p_data[int(x) - int(iso_w)] + p_data[x] * 2 + p_data[x + iso_w]) / 4;
			p_data += iso_w;
		}
	}
#endif
}

void water_to_render_geometry() {
	water_iso->Clear();
	PolygoniseIsoSurfaceToRenderGeometry(g_plus->GetRenderSystem(), water_geo, water_mat, iso_w - 2, iso_h - 2, iso_d - 2, water_field.data(), 1, water_iso, iso_unit);

	//	log(stringify("%1 indexes").arg(water_geo->display_list[0].idx_count));
}

//
void debug_particle_field(core::SimpleGraphicSceneOverlay &gfx) {
	//	gfx.SetDepthTest(false);

	auto count = particles.size();
	for (int i = 0; i < count; ++i) {
		auto p = (particles[i].pos - field_min) * particle_to_iso_cell * iso_scale + iso_min;
		draw_cross(gfx, p);
	}

	float h;
	Vector3 n;

	auto scale = Vector3(212, 0, 212) / field_size;

	for (float x = field_min.x; x < field_max.x; x += field_res.x / 2.f) {
		for (float z = field_min.z; z < field_max.z; z += field_res.z / 2.f) {
			particle_sample_ground(Vector3(x, 0, z), n, h);
			n *= 4.f;
			gfx.Line(x * scale.x, h, z * scale.z, x * scale.x + n.x, h + n.y, z * scale.z + n.z, Color::Red, Color::Yellow);
		}
	}

	//	gfx.SetDepthTest(true);
}

//
bool fast_background_simulation = false;

core::sScene scn;
core::sNode sunlight, backlight;
core::sNode light_cycle_control;
core::sNode cam;

std::function<bool()> game_state, next_game_state;

input::sDevice mouse, keyboard;

std::shared_ptr<core::SimpleGraphicSceneOverlay> gfx, ui_gfx;

std::shared_ptr<core::RenderableSystem> renderable_system;

bool main_menu_idle();
bool day_prelude();

// GAME STATE
int current_day = 1;

float get_health() {
	auto health = get_homes_energy() * 100.f / total_homes_energy;
	if (health < 0)
		health = 0;
	return health;
}

void draw_game_state_ui() {
	float health = get_health();

	const char *count_bg_path = "Peon counter 0.png";

	if (health > 70)
		count_bg_path = "Peon counter 100-71.png";
	else if (health > 50)
		count_bg_path = "Peon counter 70-51.png";
	else if (health > 30)
		count_bg_path = "Peon counter 50-31.png";
	else if (health > 10)
		count_bg_path = "Peon counter 30-11.png";
	else if (health > 0)
		count_bg_path = "Peon counter 10-01.png";

	g_plus->Image2D(40, 550, 0.75f, count_bg_path);
	g_plus->Image2D(70, 570, 0.75f, health > 0.f ? "Peon Token Clear.png" : "Peon Token Dead.png");

	g_plus->Text2D(200, 600, std::to_string(int(health)).c_str(), 90.f, Color::White, "Carton_Six.ttf");

	for (int i = 0; i < (3 - active_totems); ++i)
		g_plus->Image2D(30 + i * 58, 70, 1.f, "Totem.png");
}

//
bool night_cycle() {
	active_totems = 0;

	draw_game_state_ui();

	auto trs = light_cycle_control->GetComponent<core::Transform>();
	auto rot = trs->GetRotation();

	apply_wave(0.001f);

	rot.x += 0.075f;
	if (rot.x > units::Deg(360.f)) {
		rot.x = 0.f;
		trs->SetRotation(rot);
		next_game_state = day_prelude;
		++current_day;
		return true;
	}

	trs->SetRotation(rot);

	return false;
}

//
int game_over_delay = 60;

bool game_over() {
	g_plus->Image2D(0, 0, 1.f, "Game over 001.jpg");

	if (--game_over_delay == 0) {
		next_game_state = main_menu_idle;
		game_over_delay = 60;
		reset_homes_energy();
		return true;
	}
	return false;
}

int victory_delay = 60;

bool victory() {
	g_plus->Image2D(0, 0, 1.f, "Victory.jpg");

	if (--game_over_delay == 0) {
		next_game_state = main_menu_idle;
		victory_delay = 60;
		reset_homes_energy();
		return true;
	}
	return false;
}

//
int force_timeout = 32;
int flood_duration = 0;

float last_wave_health;

bool run_wave() {
	auto health = get_health();

	draw_game_state_ui();

	take_damage = flood_duration < 150;
	log(stringify("damage_t: %1").arg(flood_duration));

	//	if (--force_timeout > 0)
	//		apply_wave(-0.005f);

	if (flood_duration > 175) {
		auto death_count = int(last_wave_health) - int(health);

		if (death_count > 0) {
			g_plus->Image2D(0, 0, 1.f, "Deads of the day.jpg");
			g_plus->Text2D(300, 600, stringify("%1 lost their lives today").arg(death_count), 96.f, Color::White, "Carton_Six.ttf");
		}
		else {
			g_plus->Image2D(0, 0, 1.f, "fuck.jpg");
		}
	}

	if (flood_duration > 230) {
		if (!health) {
			next_game_state = game_over;
		}
		else {
			next_game_state = current_day < 3 ? night_cycle : victory;
		}

		flood_duration = 0;
		force_timeout = 32;
		return true;
	}

	++flood_duration;
	return false;
}

//
int incoming_t = 0;

void display_totem_instructions() {
	g_plus->Text2D(220, 70, "Place 3 totems to counter the flood and protect your people!", 32.f, Color::White, "Carton_Six.ttf");
}

bool incoming() {
	apply_wave(0.005f);
	display_totem_instructions();

	if (incoming_t < 20) // small timing
		;
	else if (incoming_t < 40)
		g_plus->Image2D(0, 0, 1.f, "Pointage de doigt 01.jpg");
	else if (incoming_t < 60)
		g_plus->Image2D(0, 0, 1.f, "Pointage de doigt 02.jpg");
	else if (incoming_t < 70)
		;
	else {
		next_game_state = run_wave;
		incoming_t = 0;
		return true;
	}

	++incoming_t;
	return false;
}

//
render::sGeometry green_disk, red_disk;

bool is_totem_position_valid(const Vector3 &wp) {
	Vector3 p[3] = { wp + Vector3(0, 0, 1), wp + Vector3(-1, 0, -1), wp + Vector3(1, 0, -1) };

	Vector3 n[3];
	float h[3];
	for (uint i = 0; i < 3; ++i)
		particle_sample_ground(world_to_field(p[i]), n[i], h[i]);

	static const float k_coherency_constraint = 0.9f;

	if (n[0].Dot(n[1]) < k_coherency_constraint ||
		n[1].Dot(n[2]) < k_coherency_constraint ||
		n[0].Dot(n[2]) < k_coherency_constraint)
		return false;

	return true;
}

bool place_totems() {
	draw_game_state_ui();

	apply_wave(0.005f);
	display_totem_instructions();

	//
	float mx, my;
	g_plus->GetMousePos(&mx, &my);

	Vector3 wp;
	//	if (scene_picking->Prepare(*scn, false, true).get())
	if (scene_picking->PickWorld(*scn, mx, my, wp)) {
		auto fp = world_to_field(wp);

		float h;
		Vector3 n;
		particle_sample_ground(fp, n, h);

		//
		bool is_totem_pos_valid = is_totem_position_valid(wp);

		Vector3 disk_wp = wp + n;
		auto disk_matrix = Matrix4::TransformationMatrix(disk_wp, Matrix3::LookAt(n) * Matrix3::RotationMatrixXAxis(units::Deg(90.f)));

		gfx->Line(disk_wp.x, disk_wp.y, disk_wp.z, disk_wp.x + n.x * 5.f, disk_wp.y + n.y * 5.f, disk_wp.z + n.z * 5.f, Color::White, Color::Green);

		renderable_system->DrawGeometry(is_totem_pos_valid ? green_disk : red_disk, disk_matrix);

		if (is_totem_pos_valid)
			if (mouse->WasButtonPressed(input::Device::Button0)) {
				totems[active_totems].pos = wp;
				++active_totems;
			}
	}

	//
	if (active_totems == 3) { // (keyboard->WasPressed(input::Device::KeySpace)) {
		last_wave_health = get_health();
		next_game_state = incoming;
		force_timeout = 32;
		return true;
	}
	return false;
}

// DAY PRELUDE
int prelude_timeout = 48;

bool day_prelude() {
	draw_game_state_ui();

	apply_wave(0.003f);

	auto day_title = stringify("DAY %1").arg(current_day);

	g_plus->Text2D(500, 320, day_title, 128.f, Color::White, "Carton_Six.ttf");

	active_totems = 0;

	if (--prelude_timeout == 0) {
		prelude_timeout = 48;
		next_game_state = place_totems;
		return true;
	}

	return false;
}

// TITLE SCREEN
float title_a = 0.f;

void draw_title(float offset_bg = 0.f) {
	g_plus->Image2D(0, -offset_bg, 1280.f / 720.f, "title_bg.png");

	float ox = math::Sin(title_a * -1.1f) * math::Cos(title_a * 2.f) * 10.f;
	float oy = math::Sin(title_a * 1.5f) * math::Cos(title_a * -1.2f) * 10.f;
	title_a += 0.05f;

	g_plus->Image2D(140 + ox, 177 + oy - offset_bg, 1000.f / 1920.f, "title.png");
}

float main_menu_out_t = 0;

bool main_menu_out() {
	apply_wave(0.005f);

	auto offset_bg = math::Pow(main_menu_out_t, 1.75f);
	draw_title(offset_bg);

	main_menu_out_t += 1.f;

	log(stringify("t: %1").arg(main_menu_out_t));

	if (main_menu_out_t > 60.f)
		fast_background_simulation = false;

	if (main_menu_out_t > 120.f) {
		next_game_state = day_prelude;
		main_menu_out_t = 0;
		return true;
	}

	return false;
}

int press_space_t = 0;

bool main_menu_idle() {
	fast_background_simulation = true;

	apply_wave(0.005f);

	draw_title();

	if ((press_space_t / 12) & 1)
		g_plus->Text2D(590, 100, "Press Space", 32.f, Color::White, "Carton_Six.ttf");
	++press_space_t;

	if (keyboard->WasPressed(input::Device::KeySpace)) {
		current_day = 1;

		next_game_state = main_menu_out;
		return true;
	}
	return false;
}

//
void init_lighting() {
	sunlight = scn->GetNode("Soleil");
	backlight = scn->GetNode("Backlight_ciel");

	light_cycle_control = std::make_shared<core::Node>();
	light_cycle_control->AddComponent(std::make_shared<core::Transform>());

	sunlight->GetComponent<core::Transform>()->SetParent(light_cycle_control);
	backlight->GetComponent<core::Transform>()->SetParent(light_cycle_control);

	auto l = sunlight->GetComponent<core::Light>();
	l->SetSpecularIntensity(1);
	l->SetShadowRange(400.f);
	l->SetShadow(core::Light::Shadow_Map);

	scn->AddNode(light_cycle_control);
}

//#define PACKED

//
void main(int argc, const char **argv) {
	core::Init(argv[0]);
	core::LoadPlugins();

	g_plus->RenderInit(1280, 720, 8);
#ifdef PACKED
	g_fs->Mount(std::make_shared<io::CFile>(), "@sys/");
	auto zip_h = g_fs->Open("@sys/data.zip");
	__RASSERT_MSG__(zip_h, "WWTFBBQ: Missing data.zip archive");
	g_fs->Mount(std::make_shared<io::Zip>(zip_h));
#else
	g_plus->MountFilePath("c:/gs-users/ggj2017/data");
#endif

	scn = g_plus->NewScene(false, false);
	cam = g_plus->AddCamera(*scn);

	Vector3 cam_pos(-177.6033, 216.2479, 144.5593), cam_rot(0.6950, -4.135, 0);

	//#define USE_FPS

	{
		auto trs = cam->GetComponent<core::Transform>();
		trs->SetPosition(cam_pos);
		trs->SetRotation(cam_rot);

		auto ccm = cam->GetComponent<core::Camera>();
#if 0
		ccm->SetOrthographic(true);
		ccm->SetOrthographicSize(75.f);
#else
		ccm->SetZoomFactor(8.3);
		ccm->SetZNear(1);
		ccm->SetZFar(10000);
#endif
	}

	//
	core::SceneDeserializationContext ctx(g_plus->GetRenderSystem());
	LoadResourceFromPath("terrain/terrain.scn", *scn, gs::DocumentFormatUnknown, &ctx);

	for (uint i = 0; i < 8; ++i)
		g_plus->UpdateScene(*scn, gs::time(1.f / 60.f)); // commit load

	//
	scene_picking = new core::ScenePicking(g_plus->GetRenderSystem());
	scene_picking->Prepare(scn, false, true);

	//
	g_fs->FileLoad("height.raw", heightmap);

	//
	//	g_plus->AddLight(scn, Matrix4::RotationMatrix(Vector3(0.6, -0.4, 0)), core::Light::Model_Linear, 300);

#ifdef USE_FPS
	FPSController fps;
#endif

	gfx = std::make_shared<core::SimpleGraphicSceneOverlay>(false);
	scn->AddComponent(gfx);

	renderable_system = scn->GetSystem<core::RenderableSystem>();

	g_plus->SetBlend2D(render::BlendAlpha);

	//
	green_disk = g_plus->GetRenderSystem()->LoadGeometry("totem/disque_vert.geo");
	red_disk = g_plus->GetRenderSystem()->LoadGeometry("totem/disque_rouge.geo");

	auto totem = g_plus->GetRenderSystem()->LoadGeometry("totem/totem.geo");

	//
	create_particle_field();

	init_water();
	init_lighting();

	spawn_homes(*scn);

	//
	mouse = g_plus->GetMouse();
	keyboard = g_plus->GetKeyboard();

	//
	bool visualize_particles = false;
	bool update_iso_surface = true;
	bool display_iso_surface = true;

#ifdef PACKED
	game_state = &main_menu_idle;
#else
	game_state = &place_totems;
#endif

	while (!g_plus->IsAppEnded()) {
		// -- DEBUG UI
#ifndef PACKED
		ImGui::Begin("Debug");
		ImGui::Checkbox("Visualize fluid particles", &visualize_particles);
		ImGui::Checkbox("Update iso surface", &update_iso_surface);
		ImGui::Checkbox("Display iso surface", &display_iso_surface);
		ImGui::End();
#endif

		//-- GAME CONSTANT
		auto dt = g_plus->UpdateClock();
#ifdef USE_FPS
		fps.UpdateAndApplyToNode(cam, dt);
#endif

		update_particle_field();
		if (visualize_particles)
			debug_particle_field(*gfx);

		if (!fast_background_simulation) {
			if (update_iso_surface) {
				particles_to_iso_field();
				water_to_render_geometry();
			}

			if (display_iso_surface)
				renderable_system->DrawGeometry(water_geo, Matrix4::TranslationMatrix(iso_min));
		}

		//-- TOTEMS
		for (uint i = 0; i < active_totems; ++i)
			renderable_system->DrawGeometry(totem, Matrix4::TransformationMatrix(totems[i].pos, Vector3::Zero, Vector3(3, 3, 3)));

		//-- UPDATE SCENE
		g_plus->UpdateScene(*scn, dt);

		//-- GAME STATE
		if (game_state())
			game_state = next_game_state;

		g_plus->Flip();
	}

	core::Uninit();
}
