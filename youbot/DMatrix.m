classdef DMatrix
    properties
        offset_x
        offset_y
        max_x
        max_y
        coords_x
        coords_y
        values
        matrix
    end

    methods
        function obj = DMatrix()
            obj.offset_x = 0;
            obj.offset_y = 0;
            obj.max_x = 0;
            obj.max_y = 0;
        end
        function obj = add(obj, coord, value)
            if ~isnan(value)
                changed = false;

                if coord(1) + obj.offset_x < 1
                    dx = 1 - coord(1) - obj.offset_x;
                    if ~isempty(obj.values)
                        obj.coords_x = obj.coords_x + dx;
                        obj.max_x = obj.max_x + dx;
                    end
                    obj.offset_x = obj.offset_x + dx;
                    changed = true;
                end
                if coord(2) + obj.offset_y < 1
                    dy = 1 - coord(2) - obj.offset_y;
                    if ~isempty(obj.values)
                        obj.coords_y = obj.coords_y + dy;
                        obj.max_y = obj.max_y + dy;
                    end
                    obj.offset_y = obj.offset_y + dy;
                    changed = true;
                end

                coord(1) = coord(1) + obj.offset_x;
                coord(2) = coord(2) + obj.offset_y;
                obj.coords_x = [obj.coords_x coord(1)];
                obj.coords_y = [obj.coords_y coord(2)];
                obj.values = [obj.values value];

                if coord(1) > obj.max_x
                    obj.max_x = coord(1);
                    changed = true;
                end
                if coord(2) > obj.max_y
                    obj.max_y = coord(2);
                    changed = true;
                end

                if changed
                    obj.matrix = NaN(obj.max_x, obj.max_y, 2);
                    lin_i = sub2ind(size(obj.matrix), obj.coords_x, obj.coords_y, ones(size(obj.values)));
                    lin_j = sub2ind(size(obj.matrix), obj.coords_x, obj.coords_y, 2 * ones(size(obj.values)));
                    obj.matrix(lin_i) = obj.values;
                    obj.matrix(lin_j) = 1:length(obj.values);
                else
                    obj.matrix(coord(1), coord(2), 1) = value;
                    obj.matrix(coord(1), coord(2), 2) = length(obj.values);
                end
            end
        end
        function obj = set(obj, coord, value)
            if ~isnan(value)
                coord(1) = coord(1) + obj.offset_x;
                coord(2) = coord(2) + obj.offset_y;

                if coord(1) > 0 && coord(1) <= obj.max_x && coord(2) > 0 && coord(2) <= obj.max_y
                    obj.matrix(coord(1), coord(2), 1) = value;
                    obj.values(obj.matrix(coord(1), coord(2), 2)) = value;
                end
            end
        end
        function [ret, value] = get(obj, coord)
            coord(1) = coord(1) + obj.offset_x;
            coord(2) = coord(2) + obj.offset_y;

            if coord(1) > 0 && coord(1) <= obj.max_x && coord(2) > 0 && coord(2) <= obj.max_y
                value = obj.matrix(coord(1), coord(2), 1);
                ret = true;
            else
                value = NaN;
                ret = false;
            end
        end
        function [ret, value] = getAll(obj, coord)
           ret = zeros(size(coord, 2), 1);
           value = zeros(size(coord, 2), 1);
           for i = 1 : size(coord, 2)
             c = coord(:, i);
             [r, v] = obj.get(c);
             ret(i) = r;
             value(i) = v;
          end
        end
        function obj = suggest(obj, coord, value)
            cx = coord(1) + obj.offset_x;
            cy = coord(2) + obj.offset_y;

            if cx > 0 && cx <= obj.max_x && cy > 0 && cy <= obj.max_y
               [~, vget] = obj.get(coord);
                if isnan(vget)
                    obj = obj.add(coord, value);
                else
                    obj = obj.set(coord, value);
                end
            else
                obj = obj.add(coord, value);
            end
        end
        function res = containsAll(obj, coord)
            res = true;
            for i = 1 : size(coord, 2)
                c = coord(:, i);
                [b, v] = obj.get(c);
                res = res && b && ~isnan(v);
            end
        end
    end
end
